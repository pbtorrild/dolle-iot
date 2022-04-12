#ifndef TERMINAL_MSGS_H
#define TERMINAL_MSGS_H

#include <string>
#include <stdio.h>
#include <iostream>
namespace terminal_msgs
{
	void error(std::string in) {
		std::cout << "[ERROR]: " + in << std::endl; //
	}
	void warning(std::string in) {
		std::cout << "[WARN]: " +  in << std::endl; //
	}
	void info(std::string in) {
		std::cout << "[INFO]: " +  in << std::endl; //
	}
	void in(std::string in) {
		std::cout << "[RECEIVED]: " + in << std::endl; //
	}

	size_t debug_counter=0;
	void debug(){
		std::cout << "[DEBUG]: Itteration #" + std::to_string(debug_counter)  << std::endl; //
		debug_counter++;
	}

	void debug(std::string FILE,size_t LINE){
		std::cout << "[DEBUG]: Reached line " + std::to_string(LINE) + " in " + FILE << std::endl; // 
	}
}

#endif