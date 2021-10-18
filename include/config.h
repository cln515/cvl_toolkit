#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>

namespace cvl_toolkit {

	class config {
	public:
		config(std::string inputJson);

		nlohmann::json conf;

	};
}