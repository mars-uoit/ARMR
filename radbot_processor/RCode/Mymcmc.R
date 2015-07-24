library(MCMCpack)
library(mc2d)

radbotlogpost <- function(predic, obs, ranges){
  
  getpreds <- function(predic,obs,ranges){
    intPos = rep(0, length(obs$obsX))
    radius <- matrix(NA, nrow = length(obs$obsX), ncol = ranges$numSrc)
    for (i in 1:length(obs$obsX))
    {for (j in 1:ranges$numSrc){
      radius[i,j] = ((obs$obsX[i]-predic[1,j])^2+(obs$obsY[i]-predic[2,j])^2)^(1/2)
      intPos[i]=intPos[i] + predic[3,j]*(radius[i,j]^(-2))
    }
    }
    return(intPos)
  }
  
  radbotlogprior <-function(predic, tau, ranges){
    SrcXprior = dunif(predic[1,], min=ranges$minX, max=ranges$maxX, log = TRUE)
    SrcYprior = dunif(predic[2,], min=ranges$minY, max=ranges$maxY, log = TRUE)
    SrcIntprior = dunif(predic[3,], min=0, max=ranges$maxInt, log = TRUE)
    tauprior = dtriang(tau, min=0, mode=0, max=ranges$maxTau, log = TRUE)
    return(sum(SrcXprior)+sum(SrcYprior)+sum(SrcIntprior)+tauprior)
  }
  
  radbotloglike <-function(predic, tau, obs, ranges){
    val = 0
    for (i in 1 : ranges$numSrc)
    {    
      preds = getpreds(predic, obs, ranges)
      for (j in 1 : length(obs$obsX))
      {   
        if(!is.na(obs$obsInt[j])){
          val = val + dnorm(obs$obsInt[j], mean=preds[j], sd=tau, log = TRUE)
        }
      }
    }
    return(val)
  }
  predictions <- matrix(predic[1:(length(predic)-1)], ncol = ranges$numSrc, nrow = 3)
  tau = predic[length(predic)]
  
  log.prior <-radbotlogprior(predictions, tau, ranges)
  if(is.finite(log.prior)){
    log.like <-radbotloglike(predictions, tau, obs, ranges)
    return(log.like+log.prior)
  }
  else{return(-Inf)}
}

rep.col<-function(x,n){
  matrix(rep(x,each=n), ncol=n, byrow=TRUE)
}

#startMCMC
OBS <- read.csv("Rdata2.csv")
Ranges <- data.frame(maxX=0.0, maxY=0.0, minX=0.0, minY=0.0, spanX=0.0, spanY=0.0, maxInt=0.0, maxTau=0.0, numSrc=0)

Ranges$maxX <- max(OBS$obsX)+abs(max(OBS$obsX)*.15)
Ranges$maxY <- max(OBS$obsY)+abs(max(OBS$obsY)*.15)
Ranges$minX <- min(OBS$obsX)-abs(min(OBS$obsX)*.15)
Ranges$minY <- min(OBS$obsY)-abs(min(OBS$obsY)*.15)

Ranges$spanX <- Ranges$maxX-Ranges$minX
Ranges$spanY <- Ranges$maxY-Ranges$minY

Ranges$maxInt = 1000000
Ranges$maxTau = 50000
Ranges$numSrc = 2

#initial values and tuning for X Y and Int.  These values repeat from multiple sources.  tau is appended to the end
#init = c(Ranges$spanX/2+Ranges$minX, Ranges$spanY/2+Ranges$minY, Ranges$maxInt/2)

attempts <- 0
post.radbot <- NULL
while ( is.null(post.radbot)  && attempts <= 100){
  attempts <- attempts +1
  init <-NULL
  for (i in 1:Ranges$numSrc)
  {
    SrcXinit = runif(1, min=Ranges$minX, max=Ranges$maxX)
    SrcYinit = runif(1, min=Ranges$minY, max=Ranges$maxY)
    SrcIntinit = runif(1, min=0, max=Ranges$maxInt)
    init <- c(init, SrcXinit, SrcYinit, SrcIntinit)  
  }
  init <-c(init,rtriang(1, min=0, mode=0, max=Ranges$maxTau))
  
  tune = c(.7, .7, .7)
  
  
  try(
    post.radbot <- MCMCmetrop1R(radbotlogpost, theta.init=init,
                                thin=10, mcmc=3000, burnin=3000,
                                tune=c(rep.col(tune, Ranges$numSrc), .5),
                                verbose=1000, logfun=TRUE, force.samp=FALSE, obs=OBS, ranges=Ranges)
  )
}
print(summary(post.radbot))






valDiffs <-function(vals){
  intensities <- rep(0.0, length(vals$valX))
  for (i in 1:Ranges$numSrc)
  {
  SrcX = mean(post.radbot[,1+3*(i-1)])
  SrcY = mean(post.radbot[,2+3*(i-1)])
  SrcInt = mean(post.radbot[,3+3*(i-1)])
  radius <- ((SrcX-vals$valX)^2+(SrcY-vals$valY)^2)^.5
  intensities <- intensities + SrcInt*(radius^-2)
  }
  return((intensities-vals$valInt)/vals$valInt)
}

plot.pos <- function(post, numSrc){
  plot(OBS$obsX, OBS$obsY,asp=1)
  points(c(mean(post.radbot[,1]), mean(post.radbot[,4])),y = c(mean(post.radbot[,2]),mean(post.radbot[,5])))
}
