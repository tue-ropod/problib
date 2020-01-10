/*
 * Hybrid.h
 *
 *  Created on: Sep 20, 2011
 *      Author: sdries
 */

#ifndef PROBLIB_HYBRIDPDF_H_
#define PROBLIB_HYBRIDPDF_H_

#include "PDF.h"
#include "problib/pdfs/Gaussian.h"

//#include <cassert>
 
// Use (void) to silent unused warnings.
//#define assertm(exp, msg) assert(((void)msg, exp))
namespace pbl {

class Hybrid: public PDF {

public:

    Hybrid();

    Hybrid(const Hybrid& orig);

    virtual ~Hybrid();
    
    struct distributionStruct {
         std::shared_ptr<PDF> pdf; 
         double weight;       
        };

    Hybrid& operator=(const Hybrid& other);

    //std::shared_ptr<Hybrid> clone() const;
    
    std::shared_ptr<PDF> clone() const{ return CloneMethod(); };
   
    std::shared_ptr<Hybrid> CloneMethod() const {
            //std::cout << "CLONING" << std::endl;
    return std::make_shared< Hybrid>(*this);}

    virtual double getLikelihood(std::shared_ptr<const PDF> pdf) const;

    void clear();

    double getMaxDensity() const;

    void addPDF(const PDF& pdf, double priority);

    const std::vector<distributionStruct>& getPDFS() const;

    std::string toString(const std::string& indent = "") const;


        
protected:        
    struct HybridStruct {

        std::vector<distributionStruct> pdfDistribution_;
        
        std::vector<double> weights_;

	//	int n_ptrs_;

        HybridStruct() { }

        HybridStruct(const HybridStruct& orig) {

            //for (std::vector<PDF*>::const_iterator it_pdf = orig.pdfs_.begin(); it_pdf != orig.pdfs_.end(); ++it_pdf) {
                for (std::vector< distributionStruct>::const_iterator it_pdf = orig.pdfDistribution_.begin(); it_pdf != orig.pdfDistribution_.end(); ++it_pdf) {
                        distributionStruct dist;
                        dist.pdf = it_pdf->pdf->clone();
                        dist.weight = it_pdf->weight;
                        pdfDistribution_.push_back(dist);
			}
		}

        ~HybridStruct() {
           /* for (std::vector<PDF*>::const_iterator it_pdf = pdfs_.begin(); it_pdf != pdfs_.end(); ++it_pdf) {
				delete *it_pdf;
			}
			*/
		}
	};
        
       

    std::shared_ptr<HybridStruct> ptr_;

	void cloneStruct();

};

}

#endif /* HYBRIDPDF_H_ */
