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

#include "problib/pdfs/Uniform.h"
#include "problib/conversions.h"

using namespace pbl;

Uniform::Uniform(int dim) : PDF(dim, PDF::UNIFORM), uniform_probability_(0), size_is_set_(false) {
}

Uniform::Uniform(int dim, double density) : PDF(dim, PDF::UNIFORM), uniform_probability_(density), size_is_set_(false) {
}

Uniform::Uniform(pbl::Vector mean, pbl::Vector size) : PDF(mean.size(), PDF::UNIFORM), mean_(mean), size_(size), size_is_set_(true) {
        std::cout << "Construct Uniform " << std::endl;
    calculateUniformDensity();
        std::cout << "End of Construct Uniform " << std::endl;
}

Uniform::Uniform(const Uniform& orig) : PDF(orig), mean_(orig.mean_), size_(orig.size_),
    uniform_probability_(orig.uniform_probability_), size_is_set_(orig.size_is_set_) {
}

Uniform::~Uniform() {
}

Uniform& Uniform::operator=(const Uniform& other) {
    if (this != &other)  {
        mean_ = other.mean_;
        size_ = other.size_;
        size_is_set_ = other.size_is_set_;
    	uniform_probability_ = other.uniform_probability_;
    	dimensions_ = other.dimensions_;
    }
    return *this;
}

/*std::shared_ptr<Uniform> Uniform::clone() const {
        std::shared_ptr p = std::make_shared<Uniform>(*this);
	return p;
}*/

double Uniform::getLikelihood(std::shared_ptr<const PDF> pdf) const {
	//assert_msg(false, "Uniform PDF: getLikelihood(PDF) is currently not implemented");

    assert(dimensions() == pdf->dimensions());

    if (size_is_set_) {

        Eigen::VectorXd my_min = mean_ - size_ / 2;
        Eigen::VectorXd my_max = mean_ + size_ / 2;

        if (pdf->type() == PDF::UNIFORM) {
            std::shared_ptr<const Uniform> U = pbl::PDFtoUniform(pdf);

            Eigen::VectorXd other_min = U->mean_ - U->size_ / 2;
            Eigen::VectorXd other_max = U->mean_ + U->size_ / 2;

            double overlapping_volume = 1;
            for(int i = 0; i < dimensions(); ++i) {
                double diff = std::min(my_max(i), other_max(i)) - std::max(my_min(i), other_min(i));
                if (diff <= 0) {
                    return 0;
                }
                overlapping_volume *= diff;
            }

            return overlapping_volume * uniform_probability_ * U->uniform_probability_;
        } else if (pdf->type() == PDF::HYBRID) {
                
               std::shared_ptr<const Uniform> test = shared_from_this();

            return pdf->getLikelihood(test);
        } else {
            Eigen::VectorXd other_mean;
            if (!pdf->getExpectedValue(other_mean)) {
                std::cout << pdf->toString() << std::endl;
                assert_msg(false, "Uniform likelihood calculation: cannot determine expected value of pdf.");
                return 0;
            }

            for(int i = 0; i < dimensions(); ++i) {
                if (other_mean(i) < my_min(i) || other_mean(i) > my_max(i)) {
                    return 0;
                }
            }

            return uniform_probability_;
        }
    }

    return uniform_probability_;
}

void Uniform::setDensity(const double& density) {
	uniform_probability_ = density;
    size_is_set_ = false;
}

double Uniform::getDensity(const Eigen::VectorXd& vec) const {
	return uniform_probability_;
}

double Uniform::getMaxDensity() const {
	return uniform_probability_;
}

void Uniform::setMean(const pbl::Vector mean) {
    mean_ = mean;
}

void Uniform::setSize(const pbl::Vector size) {
    size_ = size;
    calculateUniformDensity();
}

void Uniform::calculateUniformDensity() {
        std::cout << "calculateUniformDensity" << std::endl;
    double volume = 1;
      std::cout << "calculateUniformDensity 1" << std::endl;
    
    for(unsigned int i = 0; i < size_.size(); ++i) {
        volume *= size_(i);
    }
      std::cout << "calculateUniformDensity2" << std::endl;
    
    uniform_probability_ = 1.0 / volume;
      std::cout << "calculateUniformDensity3" << std::endl;
    
    size_is_set_ = true;
      std::cout << "End of calculateUniformDensity" << std::endl;
    
}

std::string Uniform::toString(const std::string& indent) const {
	std::stringstream s;
    s << "U(" << uniform_probability_;

    if (size_is_set_) {
        s << ", mean = " << mean_ << ", size = " << size_;
    }

    s << ")";

	return s.str();
}
