/*
 * Hybrid.h
 *
 *  Created on: Sep 20, 2011
 *      Author: sdries
 */

#ifndef PROBLIB_HYBRIDPDF_H_
#define PROBLIB_HYBRIDPDF_H_

#include "PDF.h"

namespace pbl {

class Hybrid: public PDF {

public:

    Hybrid();

    Hybrid(const Hybrid& orig);

    virtual ~Hybrid();

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

    const std::vector<std::shared_ptr<PDF>>& getPDFS() const;

	std::string toString(const std::string& indent = "") const;

protected:

    struct HybridStruct {

        std::vector<std::shared_ptr<PDF>> pdfs_;

	//	int n_ptrs_;

        HybridStruct() { }

        HybridStruct(const HybridStruct& orig) {

            //for (std::vector<PDF*>::const_iterator it_pdf = orig.pdfs_.begin(); it_pdf != orig.pdfs_.end(); ++it_pdf) {
                for (std::vector< std::shared_ptr<PDF>>::const_iterator it_pdf = orig.pdfs_.begin(); it_pdf != orig.pdfs_.end(); ++it_pdf) {
                pdfs_.push_back((*it_pdf)->clone());
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
