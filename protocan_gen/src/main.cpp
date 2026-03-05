#include <google/protobuf/compiler/plugin.h>

#include "generator.hpp"

int main(int argc, char * argv[])
{
  protocan_gen::ProtocAnGenerator generator;
  return google::protobuf::compiler::PluginMain(argc, argv, &generator);
}
