#include <config.h>

namespace cvl_toolkit {

	config::config(std::string inputJson) {
		std::ifstream ifs(inputJson);
		ifs >> conf;
		ifs.close();
	
	};

}