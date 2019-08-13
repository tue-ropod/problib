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

#ifndef PROBLIB_GAUSSIAN_H_
#define PROBLIB_GAUSSIAN_H_

#include "PDF.h"

namespace pbl {

/**
 * @author Sjoerd van den Dries
 * @date December, 2012
 * @version 1.0
 *
 * @brief This class represents a multi-variate Gaussian (Normal) distribution.
 */
class Gaussian: public PDF, public std::enable_shared_from_this<Gaussian> {

public:

    /**
     * @brief Constructs a (multi-variate) Gaussian with specific dimensionality
     * but leaves mean and covariance unspecified.
     * @param dim The dimensionality of the Gaussian
     */
	Gaussian(int dim) : PDF(dim, PDF::GAUSSIAN), ptr_(0) {
}


    /**
     * @brief Constructs a (multi-variate) Gaussian with specified mean and
     * covariance.
     * @param mean The mean vector of the Gaussian
     * @param cov The covariance matrix of the Gaussian
     */
	Gaussian(const Eigen::VectorXd& mu, const Eigen::MatrixXd& cov) : PDF(mu.size(), PDF::GAUSSIAN), ptr_(std::make_shared<GaussianStruct>(mu, cov)) {
}

    /**
     * @brief Copy constructor
     */
	Gaussian(const Gaussian& orig);
        //Gaussian(const Gaussian& orig) : PDF(orig.ptr_->mu_.size(), PDF::GAUSSIAN), ptr_(orig.ptr_) {
    //if (ptr_) {
    //    ++ptr_->n_ptrs_;
    //}
//}


    /**
     * @brief Destructor
     */
	virtual ~Gaussian();

    /**
     * @brief Assignment operator. The operation is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
	Gaussian& operator=(const Gaussian& other);

    /**
     * @brief Creates a clone of the object. The clone method is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
    
    std::shared_ptr<PDF> clone() const{ return CloneMethod(); };
    
    std::shared_ptr<Gaussian> CloneMethod() const { 
            
      //      Gaussian* Gtest = new Gaussian(*this);

            
            std::shared_ptr<Gaussian> G = std::make_shared< Gaussian>(*this);
            
            return G;
}

    std::shared_ptr<Gaussian> cloneThis() const {             
            return std::make_shared< Gaussian>(*this);
}
   
    
    /*  std::shared_ptr< Gaussian > Clone() const {
        std::cout << "Derived::Clone\n";
        return std::static_pointer_cast< Gaussian >(CloneImplementation());
     }
*/
	double getLikelihood(std::shared_ptr<const PDF> pdf) const;

    /**
     * @brief Calculates the density of the Gaussian at point v.
     * @param v The point to calculate the density for
     * @param max_mah_dist
     * @return The density of the Gaussian at point v.
     */
	double getDensity(const Eigen::VectorXd& v, double max_mah_dist = 0) const;

	double getDensity(const Gaussian& npdf, double max_mah_dist = 0) const;

    /**
     * @brief Calculates the maximum density of the Gaussian, i.e., the density at
     * the mean.
     * @return The maximum density of the Gaussian.
     */
	double getMaxDensity() const;

    /**
     * @brief Returns the expected value E[x] of the Gaussian, which corresponds to
     * its mean.
     * @param v The returned expected value
     * @return Always true
     */
	bool getExpectedValue(Eigen::VectorXd& v) const;

    /**
     * @brief Sets the mean of the Gaussian
     * @param mu The mean of the Gaussian
     */
	void setMean(const Eigen::VectorXd& mu);

    /**
     * @brief Sets the covariance of the Gaussian
     * @param cov The covariance matrix of the Gaussian
     */
	void setCovariance(const Eigen::MatrixXd& cov);

    /**
     * @brief Returns the mean of the Gaussian
     * @return The mean of the Gaussian
     */
	const Eigen::VectorXd& getMean() const;

    /**
     * @brief Returns the covariance matrix of the Gaussian
     * @return The covariance matrix of the Gaussian
     */
	const Eigen::MatrixXd& getCovariance() const;

    /**
     * @brief Represents the Gaussian as a string for easier console output
     * @note Should be changed into stream operator <<
     * @return The Gaussian as string
     */
	std::string toString(const std::string& indent = "") const;
        
      // virtual std::shared_ptr< PDF > CloneImplementation() const override;
         
// virtual std::shared_ptr< PDF > CloneImplementation() const override {
//         std::cout << "Derived::CloneImplementation\n";
//         return std::shared_ptr< Gaussian >(new Gaussian(*this));
//     }
protected:

	struct GaussianStruct {

		Eigen::VectorXd mu_;

		Eigen::MatrixXd cov_;

		//int n_ptrs_;

		GaussianStruct(const Eigen::VectorXd& mu, const Eigen::MatrixXd& cov) : mu_(mu) , cov_(cov) { }

		GaussianStruct(const GaussianStruct& orig) : mu_(orig.mu_), cov_(orig.cov_) { }
	};

        
       /*  std::shared_ptr< PDF > CloneImplementation() const override {
        std::cout << "Derived::CloneImplementation\n";
        Gaussian* pG = new Gaussian(*this);
        return std::shared_ptr< Gaussian >(pG);
 }*/
        
	std::shared_ptr<GaussianStruct> ptr_;

	void cloneStruct();

	double getDensity(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2, const Eigen::MatrixXd& S, double max_mah_dist = 0) const;

#define CHECK_INITIALIZED assert_msg(ptr_, "Gaussian was not yet initialized.")

};

}

#endif /* NORMALPDF_H_ */
