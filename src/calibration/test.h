#include <nodelet/nodelet.h>



namespace example_pkg
{

    class MyNodeletClass : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
            ~MyNodeletClass();
    };

}
