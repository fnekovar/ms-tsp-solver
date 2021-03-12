#include "mstsp.h"

int main(int argc, char *argv[]) {
    if(argc > 1) {
        Mstsp mstsp_instance(argv[1]);
        return mstsp_instance.run();
    } else {
        std::cout << "No config file specified.";
        return 1;
    }
}