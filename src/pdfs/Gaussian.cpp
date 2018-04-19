/************************************************************************
 *  Copyright (C) 2012 Eindhoven University of Technology (TU/e).       *
 *  All rights reserved.                                                *
 ************************************************************************
 *  Redistribution and use in source and binary forms, with or without  *
 *  modification, are permitted provided that the following conditions  *
 *  are met:                                                            *
 *                                                                      *
 *      1.  Redistributions of source code must retain the above        *
 *          copyright notice, this list of conditions and the following *
 *          disclaimer.                                                 *
 *                                                                      *
 *      2.  Redistributions in binary form must reproduce the above     *
 *          copyright notice, this list of conditions and the following *
 *          disclaimer in the documentation and/or other materials      *
 *          provided with the distribution.                             *
 *                                                                      *
 *  THIS SOFTWARE IS PROVIDED BY TU/e "AS IS" AND ANY EXPRESS OR        *
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED      *
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
 *  ARE DISCLAIMED. IN NO EVENT SHALL TU/e OR CONTRIBUTORS BE LIABLE    *
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        *
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT   *
 *  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;     *
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF       *
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT           *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE   *
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH    *
 *  DAMAGE.                                                             *
 *                                                                      *
 *  The views and conclusions contained in the software and             *
 *  documentation are those of the authors and should not be            *
 *  interpreted as representing official policies, either expressed or  *
 *  implied, of TU/e.                                                   *
 ************************************************************************/

#include "problib/pdfs/Gaussian.h"
#include "problib/pdfs/Uniform.h"

using namespace pbl;

Gaussian::Gaussian(int dim) : PDF(dim, PDF::GAUSSIAN), ptr_(0) {
}

Gaussian::Gaussian(const Eigen::VectorXd& mu, const Eigen::MatrixXd& cov) : PDF(mu.size(), PDF::GAUSSIAN), ptr_(new GaussianStruct(mu, cov)) {
}

Gaussian::Gaussian(const Gaussian& orig) : PDF(orig.ptr_->mu_.size(), PDF::GAUSSIAN), ptr_(orig.ptr_) {
    if (ptr_) {
        ++ptr_->n_ptrs_;
    }
}

Gaussian::~Gaussian() {
	if (ptr_) {
		--ptr_->n_ptrs_;

		if (ptr_->n_ptrs_ == 0) {
			delete ptr_;
		}
	}
}

Gaussian& Gaussian::operator=(const Gaussian& other)  {
	if (this != &other)  {
		if (ptr_) {
			--ptr_->n_ptrs_;
			if (ptr_->n_ptrs_ == 0) {
				delete ptr_;
			}
		}
		ptr_ = other.ptr_;
		++ptr_->n_ptrs_;

		dimensions_ = other.dimensions_;
	}
	return *this;
}

Gaussian* Gaussian::clone() const {
	return new Gaussian(*this);
}

void Gaussian::cloneStruct() {
	if (ptr_->n_ptrs_ > 1) {
		--ptr_->n_ptrs_;
		ptr_ = new GaussianStruct(*ptr_);
	}
}

double Gaussian::getLikelihood(const PDF& pdf) const {
	CHECK_INITIALIZED
	if (pdf.type() == PDF::GAUSSIAN) {
		const Gaussian* G = static_cast<const pbl::Gaussian*>(&pdf);
		return getDensity(*G);
	} else if (pdf.type() == PDF::UNIFORM) {
		const Uniform* U = static_cast<const pbl::Uniform*>(&pdf);
        return U->getLikelihood(*this);
	}

	assert_msg(false, "Gaussian: Likelihood can only be calculated with another Gaussian or Uniform pdf.");
	
	return 0;
}

double Gaussian::getDensity(const Eigen::VectorXd& v, double max_mah_dist) const {
	CHECK_INITIALIZED
	return getDensity(v, ptr_->mu_, ptr_->cov_, max_mah_dist);
}

double Gaussian::getDensity(const Gaussian& G, double max_mah_dist) const {
	CHECK_INITIALIZED
	Eigen::MatrixXd S = G.getCovariance() + ptr_->cov_;
	return getDensity(ptr_->mu_, G.getMean(), S);
	
}

double Gaussian::getMaxDensity() const {
	CHECK_INITIALIZED
	return getDensity(ptr_->mu_, ptr_->mu_, ptr_->cov_);
}

bool Gaussian::getExpectedValue(Eigen::VectorXd& v) const {
	CHECK_INITIALIZED
	v = ptr_->mu_;
	return true;
}

double Gaussian::getDensity(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, const Eigen::MatrixXd& S, double max_mah_dist) const {
	// check dimensions
	assert(v1.size() == v2.size()  && v1.size()  == S.rows());

	double det = S.determinant();
	// covariance should have non-zero determinant
	assert(det != 0);

	// calculate difference between v1 and v2
	Eigen::VectorXd diff = v2 - v1;

	// calculate squared mahalanobis distance
	double mahalanobis_dist_sq = diff.dot(S.inverse() * diff );
	// mahalanobis distance should always be 0 or positive
	assert(mahalanobis_dist_sq >= 0);

	// threshold to 0 if maximum mahalanobis distance is exceeded
	if (max_mah_dist > 0 && mahalanobis_dist_sq > (max_mah_dist * max_mah_dist)) {
		return 0;
	}

    double pi2_pow = 1;
    for(unsigned int d = 0; d < S.rows(); ++d) {
        pi2_pow *= 2 * M_PI;
    }

	// calculate density
    double pos_sqrt_pow = 1 / sqrt(pi2_pow * det);
	return exp(-0.5 * mahalanobis_dist_sq) * pos_sqrt_pow;
}

void Gaussian::setMean(const Eigen::VectorXd& mu) {
	if (ptr_) {
		cloneStruct();
	} else {
	  int nElements = mu.rows()*mu.cols();
	  Eigen::MatrixXd newMatrix(nElements, nElements);
	  ptr_ = new GaussianStruct(mu, newMatrix.setZero());
	}

	ptr_->mu_ = mu;
}

void Gaussian::setCovariance(const Eigen::MatrixXd& cov) {
	if (ptr_) {
		cloneStruct(); 
	} else {
	  Eigen::VectorXd newMatrix;
		ptr_ = new GaussianStruct(newMatrix.setZero(cov.cols()), cov);
	}

	ptr_->cov_ = cov;
}

const Eigen::VectorXd& Gaussian::getMean() const {
	CHECK_INITIALIZED
	return ptr_->mu_;
}

const Eigen::MatrixXd& Gaussian::getCovariance() const {
	CHECK_INITIALIZED
	return ptr_->cov_;
}

std::string Gaussian::toString(const std::string& indent) const {
	if (!ptr_) {
		return "N(-)";
	}

	std::stringstream s;

	s << "N( [";
	for(int i = 0; i < dimensions_; ++i) {
		s << " " << ptr_->mu_(i);
	}

	s << "], [";
	for(int i = 0; i < dimensions_; ++i) {
		for(int j = 0; j < dimensions_; ++j) {
			s << " " << ptr_->cov_(i, j);
		}
		s <<  ";";
	}
	s << "] )";

	return s.str();
}
