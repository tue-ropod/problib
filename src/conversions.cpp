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

#include "problib/conversions.h"

namespace pbl {

void PDFtoMsg(const PDF& pdf, problib::PDF& msg) {
	msg.dimensions = pdf.dimensions();
	serialize(pdf, msg);

	if (pdf.type() == PDF::DISCRETE) {
		msg.type = problib::PDF::DISCRETE;
	} else {
		msg.type = msg.data[0];
	}
}

problib::PDF PDFtoMsg(const PDF& pdf) {
    problib::PDF msg;
    PDFtoMsg(pdf, msg);
    return msg;
}

std::shared_ptr<PDF> msgToPDF(const problib::PDF& msg) {
	int i_data = 1;
	return deserialize(msg, msg.type, i_data);
}

std::shared_ptr<Gaussian> msgToGaussian(const problib::PDF& msg) {
	int i_data = 1;
	if (msg.type == problib::PDF::GAUSSIAN) {
		return deserialize_gaussian(msg, i_data);
	}
	return 0;
}

std::shared_ptr<Mixture> msgToMixture(const problib::PDF msg) {
	int i_data = 1;
	if (msg.type == problib::PDF::MIXTURE) {
		return deserialize_mixture(msg, i_data);
	}
	return 0;
}

std::shared_ptr<PMF> msgToPMF(const problib::PDF& msg) {
	if (msg.type == problib::PDF::DISCRETE) {
		return deserialize_discrete(msg);
	}
	return 0;
}

std::shared_ptr<const Mixture> PDFtoMixture(std::shared_ptr<const PDF> pdf) {
	if (pdf->type() != PDF::MIXTURE) {
		return 0;
	}
	std::shared_ptr<const Mixture> M = std::static_pointer_cast<const Mixture>(pdf);
        
        //std::shared_ptr<T> static_pointer_cast( const std::shared_ptr<U>& r ) noexcept;
        
	return M;               
}

std::shared_ptr<const Gaussian> PDFtoGaussian(std::shared_ptr<const PDF> pdf) {
	if (pdf->type() != PDF::GAUSSIAN) {
		return 0;
	}

	std::shared_ptr<const Gaussian> G = std::static_pointer_cast<const Gaussian>(pdf);
       // const Gaussian* G = static_cast<const Gaussian*>(&pdf);
	return G;
}

std::shared_ptr<const Uniform> PDFtoUniform(std::shared_ptr<const PDF> pdf) {
	if (pdf->type() != PDF::UNIFORM) {
		return 0;
	}
	std::shared_ptr<const Uniform> U = std::static_pointer_cast<const Uniform>(pdf);
	return U;
}

std::shared_ptr<const PMF> PDFtoPMF(std::shared_ptr<const PDF> pdf) {
	if (pdf->type() != PDF::DISCRETE) {
		return 0;
	}
	
	std::shared_ptr<const PMF> P = std::static_pointer_cast<const PMF>(pdf);
	return P;
}

std::shared_ptr<const Hybrid> PDFtoHybrid(std::shared_ptr<const PDF> pdf) {
    if (pdf->type() != PDF::HYBRID) {
        return 0;
    }
    
    std::shared_ptr<const Hybrid> H = std::static_pointer_cast<const Hybrid>(pdf);
        return H;
    
    //return static_cast<std::shared_ptr<const Hybrid>>(&pdf);
}

std::string typeToName(PDF::PDFType type) {
    if (type == PDF::GAUSSIAN) {
        return "gaussian";
    } else if (type == PDF::MIXTURE) {
        return "mixture";
    } else if (type == PDF::UNIFORM) {
        return "uniform";
    } else if (type == PDF::DISCRETE) {
        return "discrete";
    } else if (type == PDF::HYBRID) {
        return "hybrid";
    }
    return "unknown";
}

/* * * * * * * SERIALIZATION AND DESERIALIZATION * * * * * * */

void serialize(const PDF& pdf, problib::PDF& msg) {
	if (pdf.type() == PDF::GAUSSIAN) {
		const Gaussian* gauss = static_cast<const Gaussian*>(&pdf);
		serialize_gaussian(*gauss, msg);
	} else if (pdf.type() == PDF::MIXTURE) {
		const Mixture* mix = static_cast<const Mixture*>(&pdf);
		serialize_mixture(*mix, msg);
	} else if (pdf.type() == PDF::UNIFORM) {
		const Uniform* uniform = static_cast<const Uniform*>(&pdf);
		serialize_uniform(*uniform, msg);
	} else if (pdf.type() == PDF::DISCRETE) {
		const PMF* pmf = static_cast<const PMF*>(&pdf);
		serialize_discrete(*pmf, msg);
    } else if (pdf.type() == PDF::HYBRID) {
        const Hybrid* hybrid = static_cast<const Hybrid*>(&pdf);
        serialize_hybrid(*hybrid, msg);
    }
}

std::shared_ptr<PDF> deserialize(const problib::PDF& msg, int type, int& i_data) {
	if (type == problib::PDF::MIXTURE) {
		return deserialize_mixture(msg, i_data);
	} else if (type == problib::PDF::GAUSSIAN) {
		return deserialize_gaussian(msg, i_data);
	} else if (type == problib::PDF::UNIFORM) {
		return deserialize_uniform(msg, i_data);
	} else if (type == problib::PDF::DISCRETE) {
		return deserialize_discrete(msg);
	} else if (type == problib::PDF::EXACT) {
		return deserialize_exact(msg);
    } else 	if (type == problib::PDF::HYBRID) {
        return deserialize_hybrid(msg, i_data);
    }
	return 0;
}

void serialize_gaussian(const Gaussian& gauss, problib::PDF& msg) {
	int dimensions = gauss.dimensions();
	int new_data_size = dimensions + ((dimensions + 1) * dimensions / 2) + 1;

	msg.data.reserve(msg.data.size() + new_data_size);

	msg.type = problib::PDF::GAUSSIAN;
	msg.data.push_back(msg.type);

	const Eigen::VectorXd& mu = gauss.getMean();
	for(int i = 0; i < dimensions; ++i) {
		msg.data.push_back(mu(i));
	}

	const Eigen::MatrixXd& cov = gauss.getCovariance();
	for(int i = 0; i < dimensions; ++i) {
		for(int j = i; j < dimensions; ++j) {
			msg.data.push_back(cov(i, j));
		}
	}
}

std::shared_ptr<Gaussian> deserialize_gaussian(const problib::PDF& msg, int& i_data) {
	Eigen::VectorXd mu(msg.dimensions);
	for(unsigned int i = 0; i < msg.dimensions; ++i) {
		mu(i) = msg.data[i_data++];
	}

	Eigen::MatrixXd cov(msg.dimensions, msg.dimensions);
	for(unsigned int i = 0; i < msg.dimensions; ++i) {
		for(unsigned int j = i; j < msg.dimensions; ++j) {
			cov(i, j) = msg.data[i_data];
			cov(j, i) = msg.data[i_data];
			++i_data;
		}
	}

	return std::make_shared<Gaussian>(mu, cov);
}

void serialize_mixture(const Mixture& mix, problib::PDF& msg) {
	// add type of pdf (mixture)
	msg.data.push_back(problib::PDF::MIXTURE);

	// add number of mixture components
	msg.data.push_back(mix.components());

	// add weights and components themselves
	for(int i = 0; i < mix.components(); ++i) {
		msg.data.push_back(mix.getWeight(i));
		std::shared_ptr<const PDF> pdf = mix.getComponent(i);
		serialize(*pdf, msg);
	}
}

std::shared_ptr<Mixture> deserialize_mixture(const problib::PDF& msg, int& i_data) {
	std::shared_ptr<Mixture> mix = std::make_shared<Mixture>();

	int num_components = (int)msg.data[i_data++];

	for(int c = 0; c < num_components; ++c) {
		double w = msg.data[i_data++];
		int type = (int)msg.data[i_data++];

		std::shared_ptr<PDF> component = deserialize(msg, type, i_data);
		mix->addComponent(component, w);
		//delete component;
	}

	mix->normalizeWeights();

	return mix;
}

void serialize_uniform(const Uniform& uniform, problib::PDF& msg) {
	msg.data.push_back(problib::PDF::UNIFORM);
	msg.data.push_back(uniform.getMaxDensity());
}

std::shared_ptr<Uniform> deserialize_uniform(const problib::PDF& msg, int& i_data) {
	std::shared_ptr<Uniform> uniform = std::make_shared<Uniform>(msg.dimensions);
	uniform->setDensity(msg.data[i_data++]);
	return uniform;
}

void serialize_discrete(const PMF& pmf, problib::PDF& msg) {
	pmf.getValues(msg.values);
	pmf.getProbabilities(msg.probabilities);
	msg.domain_size = pmf.getDomainSize();
}

std::shared_ptr<PMF> deserialize_discrete(const problib::PDF& msg) {
	std::shared_ptr<PMF> pmf = std::make_shared<PMF>(msg.domain_size);
	std::vector<double>::const_iterator it_p = msg.probabilities.begin();
	for(std::vector<std::string>::const_iterator it_v = msg.values.begin(); it_v != msg.values.end(); ++it_v) {
		pmf->setProbability(*it_v, *it_p);
		++it_p;
	}
	pmf->setDomainSize(msg.domain_size);
	return pmf;
}

void serialize_hybrid(const Hybrid& hybrid, problib::PDF& msg) {
    // add type of pdf (hybrid)
    msg.data.push_back(problib::PDF::HYBRID);

    // add number of hybrid components
    msg.data.push_back(hybrid.getPDFS().size());

    // add components themselves
    const std::vector<std::shared_ptr<PDF>> pdfs = hybrid.getPDFS();
    for(std::vector<std::shared_ptr<PDF>>::const_iterator it = pdfs.begin(); it != pdfs.end(); ++it) {
        const PDF& pdf = **it;

        if (pdf.type() == PDF::DISCRETE) {
            bool already_contains_pmf = msg.domain_size != 0 || !msg.values.empty();
            assert_msg(!already_contains_pmf, "Can currently only decode at most one discrete pdf in hybrid pdf message.");
            msg.data.push_back(problib::PDF::DISCRETE);
        }

        serialize(pdf, msg);
    }
}

std::shared_ptr<Hybrid> deserialize_hybrid(const problib::PDF& msg, int& i_data) {
    std::shared_ptr<Hybrid> hybrid = std::make_shared<Hybrid>();

    int num_components = (int)msg.data[i_data++];

    for(int c = 0; c < num_components; ++c) {
        int type = (int)msg.data[i_data++];

        std::shared_ptr<PDF> pdf = deserialize(msg, type, i_data);
        hybrid->addPDF(*pdf, -1);
      //  delete pdf;
    }

    return hybrid;
}

std::shared_ptr<PDF> deserialize_exact(const problib::PDF& msg) {
	if (!msg.exact_value_vec.empty()) {
		// vector value, so we ASSUME continuous
		unsigned int dim = msg.exact_value_vec.size();
		Eigen::VectorXd mu(dim);
		for(unsigned int i = 0; i < dim; ++i) {
			mu(i) = msg.exact_value_vec[i];
		}
		Eigen::MatrixXd cov;
		cov.setZero(dim,dim);
		
		return std::make_shared<Gaussian>(mu, cov);
	} else if (msg.exact_value_str != "") {
		// string value, so discrete
		std::shared_ptr<PMF> pmf = std::make_shared<PMF>();
		pmf->setProbability(msg.exact_value_str, 1.0);
		return pmf;
	}
	// no exact value found
	return 0;
}

}
