library(MCMCpack)

radbotlogpost <- function(predic, obs, ranges){
  
  getpreds <- function(predic,obs,ranges){
    intPos = rep(0, length(obs$obsX))
    radius <- matrix(NA, nrow = length(obs$obsX), ncol = ranges$numSrc)
    for (i in 1:length(obs$obsX))
         {for (j in 1:ranges$numSrc){
           radius[i,j] = ((obs$obsX[i]-predic[j,1])^2+(obs$obsY[i]-predic[j,2])^2)^(1/2)
           intPos[i]=intPos[i] + predic[j,3]*(radius[i,j]^(-2))
         }
    }
    return(intPos)
  }
  
  radbotlogprior <-function(predic, ranges){
    SrcXprior = dunif(predic[,1], min=ranges$minX, max=ranges$maxX, log = TRUE)
    SrcYprior = dunif(predic[,2], min=ranges$minY, max=ranges$maxY, log = TRUE)
    SrcIntprior = dunif(predic[,3], min=0, max=ranges$maxInt, log = TRUE)
    tauprior = dunif(predic[1,4], min=0, max=ranges$maxTau, log = TRUE)
    return(sum(SrcXprior)+sum(SrcYprior)+sum(SrcIntprior)+sum(tauprior))
  }
  radbotloglike <-function(predic, obs, ranges){
    val = 0
    if(predic[1,4]>0){
    for (i in 1 : ranges$numSrc)
    {    
      preds = getpreds(predic, obs, ranges)
      
      for (j in 1 : length(obs$obsX))
      {   
        if(!is.na(obs$obsInt[j])){
          val = val + dnorm(obs$obsInt[j], mean=preds[j], sd=predic[1,4], log = TRUE)
        }
      }
    }
    return(val)
    }
    else{return(-Inf)}
  }
  log.like <-radbotloglike(t(predic), obs, ranges)
  log.prior <-radbotlogprior(t(predic), ranges)
  return(log.like+log.prior)
  
}

rep.col<-function(x,n){
  matrix(rep(x,each=n), ncol=n, byrow=TRUE)
}

#startMCMC
OBS <- read.csv("Rdata2.csv")
#Ranges <- as.data.frame(matrix(0.0, nrow = 1, ncol = 7))
Ranges$maxX <- max(OBS$obsX)+abs(max(OBS$obsX)*.15)
Ranges$maxY <- max(OBS$obsY)+abs(max(OBS$obsY)*.15)
Ranges$minX <- min(OBS$obsX)-abs(min(OBS$obsX)*.15)
Ranges$minY <- min(OBS$obsY)-abs(min(OBS$obsY)*.15)

Ranges$spanX <- Ranges$maxX-Ranges$minX
Ranges$spanY <- Ranges$maxY-Ranges$minY

Ranges$maxInt = 1000000
Ranges$maxTau = 50000
Ranges$numSrc = 2

init = c(Ranges$spanX/2+Ranges$minX, Ranges$spanY/2+Ranges$minY, Ranges$maxInt/2, 10000)
tune = c(.5, .5, .5, 1)

post.radbot <- MCMCmetrop1R(radbotlogpost, theta.init=rep.col(init, Ranges$numSrc),
                            thin=10, mcmc=100000, burnin=50000,
                            tune=rep.col(tune, Ranges$numSrc),
                            verbose=5000, logfun=TRUE, force.samp=TRUE, obs=OBS, ranges=Ranges)

print(summary(post.radbot))

mean

valDiffs <-function(vals){
  SrcX = mean(post.radbot[,1])
  SrcY = mean(post.radbot[,2])
  SrcInt = mean(post.radbot[,3])
  radius <- ((SrcX-vals$valX)^2+(SrcY-vals$valY)^2)^.5
  intensities <- SrcInt*(radius^-2)
  return((intensities-vals$valInt)/vals$valInt)
}
