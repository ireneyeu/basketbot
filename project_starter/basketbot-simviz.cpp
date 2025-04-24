// #include "simviz/SimVizRedisInterface.h"
// #include "simviz/SimVizConfigParser.h"

// int main(int argc, char** argv) {
// 	SaiModel::URDF_FOLDERS["BASKETBOT_URDF_FOLDER"] = std::string(BASKETBOT_URDF_FOLDER);
//     SaiModel::URDF_FOLDERS["BASKETBOT_FOLDER"] = std::string(PROJECT_STARTER);
//     std::string config_file = std::string(PROJECT_STARTER) + "/simviz_config.xml";
//     SaiInterfaces::SimVizConfigParser parser;
//     SaiInterfaces::SimVizRedisInterface simviz(parser.parseConfig(config_file));
//     simviz.run();
//     return 0;
// }


#include <filesystem>

#include "MainRedisInterface.h"


int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["BASKETBOT_URDF_FOLDER"] = std::string(BASKETBOT_URDF_FOLDER);
    SaiModel::URDF_FOLDERS["BASKETBOT_FOLDER"] = std::string(PROJECT_STARTER);
    std::string config_file = "/simviz_config.xml";

	// define the xml files folder. Only config files in that folder can be used
	// by this application
	std::string xml_files_folder = std::string(PROJECT_STARTER);
	SaiInterfaces::MainRedisInterface main_interface(xml_files_folder,
													  config_file);

	return 0;
}