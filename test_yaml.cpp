#include <iostream>
#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <yaml_file>" << std::endl;
        return 1;
    }
    
    try {
        std::cout << "Trying to load: " << argv[1] << std::endl;
        YAML::Node config = YAML::LoadFile(argv[1]);
        std::cout << "Successfully loaded YAML file!" << std::endl;
        
        if (config["camera_name"]) {
            std::cout << "camera_name: " << config["camera_name"].as<std::string>() << std::endl;
        }
        
        return 0;
    } catch (const YAML::BadFile& e) {
        std::cerr << "YAML::BadFile caught: " << e.what() << std::endl;
        return 1;
    } catch (const YAML::ParserException& e) {
        std::cerr << "YAML::ParserException caught: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "std::exception caught: " << e.what() << std::endl;
        return 1;
    }
}
