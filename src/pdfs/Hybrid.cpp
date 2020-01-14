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

#include "problib/pdfs/Hybrid.h"

using namespace pbl;

Hybrid::Hybrid() : PDF(-1, PDF::HYBRID), ptr_(0) {
}

Hybrid::Hybrid(const Hybrid& orig) : PDF(orig), ptr_(orig.ptr_) {
//     if (ptr_) {
//         ++ptr_->n_ptrs_;
//     }
}

Hybrid::~Hybrid() {
// 	if (ptr_) {
// 		--ptr_->n_ptrs_;

// 		if (ptr_->n_ptrs_ == 0) {
// 			delete ptr_;
// 		}
// 	}
}

Hybrid& Hybrid::operator=(const Hybrid& other)  {
    if (this != &other)  {
//     	if (ptr_) {
// 			--ptr_->n_ptrs_;
// 			if (ptr_->n_ptrs_ == 0) {
// 				delete ptr_;
// 			}
//     	}
    	ptr_ = other.ptr_;
//     	++ptr_->n_ptrs_;

    	dimensions_ = other.dimensions_;
    }
    return *this;
}

/*std::shared_ptr<Hybrid> Hybrid::clone() const {
        std::shared_ptr<Hybrid> p = std::make_shared<Hybrid>(*this);
    return p;
}*/

void Hybrid::cloneStruct() {
	if (ptr_.use_count() > 1) {
// 		--ptr_->n_ptrs_;
        ptr_ = std::make_shared<HybridStruct>(*ptr_);
	}
}

double Hybrid::getLikelihood(std::shared_ptr<const PDF> pdf) const { // TODO asserts do not terminate program?!
    //assert_msg(false, "Likelihood method not implemented. Please create a subclass of Hybrid and implement your own method.");
         assert_msg(ptr_, "Hybrid does not contain components.");
         assert_msg(pdf->type() == HYBRID, "Likelihood of hybrid: only hybrid can be considered.");
          
      //   std::cout << "pdf->type() = " << pdf->type() << std::endl;
         
         std::shared_ptr<const Hybrid> otherHybrid = std::static_pointer_cast<const Hybrid>(pdf);
         assert(this->getPDFS().size() == otherHybrid->getPDFS().size());
        
        // std::cout << "this->getPDFS().size() = " << this->getPDFS().size() << std::endl;
        // std::cout << "otherHybrid->getPDFS().size() = " << otherHybrid->getPDFS().size() << std::endl;
//          bool check =  this->getPDFS().size() == otherHybrid->getPDFS().size() ;
       //  std::cout << "check = " << check << std::endl;
         
       //  std::cout << "Hybrid get likelihood: other = " << otherHybrid->toString() << std::endl;
         
        double likelihood = 0.0, weightsum = 0.0;
        
        //std::cout << "Hybrid: ptr_->pdfDistribution_.size() = " << ptr_->pdfDistribution_.size() << std::endl;
        
        for (unsigned int iHyb = 0; iHyb < ptr_->pdfDistribution_.size(); iHyb++)
                //std::vector<distributionStruct>::const_iterator it_dist = ptr_->pdfDistribution_.begin(); it_dist != ptr_->pdfDistribution_.end(); ++it_dist) 
        {
//                 std::cout << "iHyb = " << iHyb << std::endl;
               /* ptr_->pdfDistribution_[iHyb].pdf;
                ptr_->pdfDistribution_[iHyb].weight;
                
                otherHybrid->getPDFS()[iHyb].pdf;
                otherHybrid->getPDFS()[iHyb].weight;
                */
                
                double weight = ptr_->pdfDistribution_[iHyb].weight*otherHybrid->getPDFS()[iHyb].weight;
                weightsum += weight;
//                 std::cout << "Hybrid: weight = " << weight << std::endl;
              /*  if(iHyb == 0 )
                {                       std::cout << "Hybrid, rectangle: this pdf = " << ptr_->pdfDistribution_[iHyb].pdf->toString() << std::endl;}
                else 
                        {                       std::cout << "Hybrid, circle: this pdf = " << ptr_->pdfDistribution_[iHyb].pdf->toString() << std::endl;}
*/
                std::shared_ptr<const PDF> thisPDF =  ptr_->pdfDistribution_[iHyb].pdf;
                std::shared_ptr<const PDF> otherPDF = otherHybrid->getPDFS()[iHyb].pdf;
                
                assert_msg(thisPDF->type() == otherPDF->type(), "Likelihood of hybrid: different pdf's can not be considered.");
                
                bool test = thisPDF->type() == GAUSSIAN ;
//                 std::cout << "thisPDF->type() = " << thisPDF->type() << " check for Gauss = " << test << std::endl;
                
                if(thisPDF->type() == GAUSSIAN)
                {
                        std::shared_ptr<const Gaussian> thisGauss = std::static_pointer_cast<const Gaussian>(thisPDF);
                        std::shared_ptr<const Gaussian> otherGauss =  std::static_pointer_cast<const Gaussian>(otherPDF);
                        
//                         bool test2 = thisGauss->getMean().size() == otherGauss->getMean().size() ;
//                         std::cout << "size = " << thisGauss->getMean().size() << ", " << otherGauss->getMean().size() << " check = " << test2 << std::endl;
                        
                        assert_msg(thisGauss->getMean().size() == otherGauss->getMean().size(), "Hybrid: unequal state dimensions."); 
                }
                
               
//                 std::cout << "Hybrid: other pdf = " << otherPDF->toString() << std::endl;
                likelihood += weight * thisPDF->getLikelihood(otherPDF);
//                 std::cout << "Hybrid: this pdf = " << thisPDF->toString() << " weight = "  << ptr_->pdfDistribution_[iHyb].weight << std::endl;
//                 std::cout << "Hybrid: other pdf = " << otherPDF->toString() << " weight = "  << otherHybrid->getPDFS()[iHyb].weight << std::endl;
//                 std::cout << "Hybrid: likelihood component = " << thisPDF->getLikelihood(otherPDF) << std::endl;
//                 std::cout << "Hybrid: weightProduct = " << weight << "\n\n" << std::endl;
        }
        
//          std::cout << "Hybrid: get likelihood: this = " << this->toString() << std::endl;
//          std::cout << "Hybrid: get likelihood: other = " << otherHybrid->toString() << std::endl;
//          std::cout << "Hybrid: likelihood, weightsum, likelihood/weightsum = " << likelihood << ", " << weightsum << ", " << likelihood/weightsum << "\n\n" << std::endl;
        
        return likelihood/weightsum; // correct (normalize) for taking both weights into consideration
        
   /*   // Mixture implementation. TODO: adopt & update with proper checks   
        assert_msg(ptr_, "Mixture does not contain components.");
        assert(ptr_->num_components_ > 0);
        assert(ptr_->weights_total_ == 1);

        double likelihood = 0;
        std::vector<double>::const_iterator it_w =ptr_-> weights_.begin();
        for (std::vector<std::shared_ptr<PDF>>::const_iterator it_pdf = ptr_->components_.begin(); it_pdf != ptr_->components_.end(); ++it_pdf) {
                likelihood += (*it_w) * (*it_pdf)->getLikelihood(pdf);
                ++it_w;
        }
        return likelihood;
     */   
}

void Hybrid::clear() {
	if (ptr_) {
// 		--ptr_->n_ptrs_;
// 		if (ptr_->n_ptrs_ == 0) {
// 			delete ptr_;
// 		}
		ptr_ = 0;
	}
}

double Hybrid::getMaxDensity() const {
    assert_msg(false, "Cannot calculate MaxDensity of Hybrid.");
	return 0;
}

void Hybrid::addPDF(const PDF& pdf, double priority) {
	if (dimensions_ < 0) {
		dimensions_ = pdf.dimensions();
	} else {
        assert(dimensions_ == pdf.dimensions() || pdf.type() == PDF::DISCRETE);
	}

	if (!ptr_) {
        ptr_ = std::make_shared<HybridStruct>();
	} else {
		cloneStruct();
	}

// 	std::cout << "Hybrid::addPDF = " << pdf.toString()<< std::endl;
	
        distributionStruct distribution;
        distribution.pdf = pdf.clone();
        distribution.weight = priority;
        
        ptr_->pdfDistribution_.push_back(distribution);
}

const std::vector<Hybrid::distributionStruct>& Hybrid::getPDFS() const {
    assert_msg(ptr_, "Hybrid does not contain pdfs.");
    return ptr_->pdfDistribution_;
}

std::string Hybrid::toString(const std::string& indent) const {
	if (!ptr_) {
        return "HYBRID(-)";
	}

	std::string new_indent = indent + "  ";

	std::stringstream ss;
    ss << "HYBRID{\n";
    for (std::vector<Hybrid::distributionStruct>::const_iterator it_pdf = ptr_->pdfDistribution_.begin(); it_pdf != ptr_->pdfDistribution_.end(); ++it_pdf) {
        ss << new_indent << it_pdf->pdf->toString(new_indent) << " weight = " << it_pdf->weight << "\n";
	}
	ss << indent << "}";
	return ss.str();
}
