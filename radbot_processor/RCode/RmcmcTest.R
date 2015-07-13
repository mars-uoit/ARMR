negbinlogpost<- function(theta, y, X, b0, B0, c0, d0){
  ## log of inverse gamma density
  logdinvgamma <- function(sigma2, a, b){
    logf <- a * log(b) - lgamma(a) + -(a+1) * log(sigma2) + -b/sigma2
    return(logf)
  }
  ## log of multivariate normal density
  logdmvnorm <- function(theta, mu, Sigma){
    d <- length(theta)
    logf <- -0.5*d * log(2*pi) - 0.5*log(det(Sigma)) -
      0.5 * t(theta - mu) %*% solve(Sigma) %*% (theta - mu)
    return(logf)
  }
  k <- length(theta)
  beta <- theta[1:(k-1)]
  alpha <- exp(theta[k])
  mu <- exp(X %*% beta)
  ## evaluate log-likelihood at (alpha, beta)
  log.like <- sum(
    lgamma(y+alpha) - lfactorial(y) - lgamma(alpha) + alpha * log(alpha/(alpha+mu)) +
      y * log(mu/(alpha+mu))
  )
  ## evaluate log prior at (alpha, beta)
  ## note Jacobian term necessary b/c of transformation
  log.prior <- logdinvgamma(alpha, c0, d0) + theta[k] +
    logdmvnorm(beta, b0, B0)
  return(log.like+log.prior)
}

library(car)
data(Ornstein)
yvec <- Ornstein$interlocks
Xmat <- model.matrix(~sector+nation, data=Ornstein)
post.negbin <- MCMCmetrop1R(negbinlogpost, theta.init=rep(0,14),
                            X=Xmat, y=yvec,
                            thin=2, mcmc=50000, burnin=1000,
                            tune=rep(.7,14),
                            verbose=500, logfun=TRUE,
                            b0=0, B0=diag(13)*1000, c0=1, d0=1)
summary(post.negbin)