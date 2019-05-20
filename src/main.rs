// extern crate choldate_rs;
extern crate nalgebra as na;
use num_traits::pow::Pow;
use choldate_rs::{choldowndate, cholupdate};
pub struct SqrtMerweScaledSigmaPoint {
    n: usize,
    alpha: f64,
    beta: f64,
    kappa: f64,
}

impl SqrtMerweScaledSigmaPoint {
    pub fn new(n: usize, alpha: f64, beta: f64, kappa: f64) -> SqrtMerweScaledSigmaPoint {
        SqrtMerweScaledSigmaPoint {
            n,
            alpha,
            beta,
            kappa,
        }
    }
    pub fn num_sigmas(&self) -> usize {
        // Fun Stuff 2 Debug!
        2 * dbg!(self.n) + 1
    }
    pub fn sigma_points(
        &self,
        x: na::base::DVector<f64>,
        mut S: na::base::DMatrix<f64>,
    ) -> na::base::DMatrix<f64> {
        assert_eq!(self.n, x.len());
        if x.len() == 1 {
            let x = x[0];
        }
        if S.len() == 1 {
            S = na::base::DMatrix::<f64>::identity(self.n, self.n)
        }
        let alpha_ = self.alpha.pow(2);
        let lambda_ = alpha_ * (self.n as f64 + self.kappa) - self.n as f64;
        let U = (lambda_ as f64).sqrt() * S;

        let mut sigmas: na::base::DMatrix<f64> = na::base::DMatrix::zeros(2 * self.n + 1, self.n);
        // The use of the x vector is horribly bad and can't possily be right
        sigmas[0] = x[0];
        for k in (0..self.n).rev() {
            sigmas[k + 1] = x[0] - (-U[k]);
            sigmas[self.n + k + 1] = x[0] - U[k];
        }
        return sigmas;
        // return x;
    }
    pub fn weights(&self) -> (na::base::DVector<f64>, na::base::DVector<f64>) {
        let lambda_ = self.alpha.pow(2) * (self.n as f64 + self.kappa) - self.n as f64;
        let c = 0.5 / (self.n as f64 + lambda_);
        let mut Wc = na::base::DVector::zeros(2 * self.n + 1);
        na::base::Matrix::fill(&mut Wc, c);
        let mut Wm = na::base::DVector::zeros(2 * self.n + 1);
        na::base::Matrix::fill(&mut Wm, c);
        Wc[0] = lambda_ / (self.n as f64 + lambda_) + (1.0 - self.alpha.pow(2) + self.beta);
        Wm[0] = lambda_ / (self.n as f64 + lambda_);
        return (Wm, Wc);
    }
}

pub fn sqrt_unscented_transform(
    sigmas: na::base::DMatrix<f64>,
    Wm: na::base::DVector<f64>,
    Wc: na::base::DVector<f64>,
    noise_cov2: na::base::DMatrix<f64>,
) -> (na::base::DVector<f64>, na::base::DMatrix<f64>) {
    let (kmax, n) = sigmas.shape();
    let x = &sigmas * Wm;
    let mut temp = na::base::DMatrix::<f64>::zeros(kmax - 1 + n, n);

    for k in 1..kmax {
        temp.set_row(k - 1, &(Wc[k].sqrt() * (&sigmas.row(k) - &x)));
    }

    for k in 0..n {
        temp.set_row(kmax + k - 1, &(noise_cov2.row(k)));
    }

    let mut r_mat = temp.qr().unpack_r();
    if Wc[0] > 0.0 {
        let mut thing = Wc[0].sqrt() * (sigmas.row(0).transpose() - &x);
        cholupdate(&mut r_mat, &mut thing);
    } else {
        let mut thing = -Wc[0].sqrt() * (sigmas.row(0).transpose() - &x);
        choldowndate(&mut r_mat, &mut thing);
    }

    (x, r_mat)
}

// pub struct SqrtUnscentedKalmanFilter<>{
//     Q2:na::base::DMatrix::<f64>,
//     R2: na::base::DMatrix::<f64>,
//     x: na::base::DVector::<f64>,
//     S: na::base::DMatrix::<f64>,
//     _dim_x: usize,
//     _dim_z: usize,
//     // points_fn = points
//     _num_sigmas: usize,
//     hx: usize,
//     fx: usize,
//     // x_mean = x_mean_fn
//     // z_mean = z_mean_fn
// }

// impl SqrtUnscentedKalmanFilter<>{
//     pub fn new() -> SqrtUnscentedKalmanFilter {
//     self.Q2 = np.eye(dim_x)
//     self.R2 = np.eye(dim_z)
//     self.x = np.zeros(dim_x)
//     self.S = np.eye(dim_x)
//     self._dim_x = dim_x
//     self._dim_z = dim_z
//     self.points_fn = points
//     self._num_sigmas = points.num_sigmas()
//     self.hx = hx
//     self.fx = fx
//     self.x_mean = x_mean_fn
//     self.z_mean = z_mean_fn
//     }

// }
pub fn kalman_pokemon_ex() -> usize {
    retun 0;
}
fn main() {
    let x = na::base::DVector::from_element(2, 0.0);
    println!("{}", x.len());
    let c = 1;
    let n = 2;
}
