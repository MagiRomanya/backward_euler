#include <iostream>
#include <string.h>

class A{
    public:
        virtual void sayHello() {
            std::cout << "hello from A" << std::endl;
        }
};

class B : public A {
    public:
        double hola, que, tal;
        void sayHello() override {
            std::cout << "hello from B" << std::endl;
        }
};

int main(int argc, char *argv[]) {
    A a;
    B b;
    A* bptr = (A*) malloc(sizeof(B));
    memcpy((void*) bptr, (void*) &b, sizeof(B));

    a.sayHello();
    b.sayHello();

    bptr->sayHello();

    std::cout << "Hello World" << std::endl;
    free(bptr);
    return 0;
}
