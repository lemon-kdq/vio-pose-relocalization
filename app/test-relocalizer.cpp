#include <vio-pose-relocalization/vio-pose-relocalization.hpp>
typedef VioPoseRelocalization::SE3 SE3F;
int main() {
  VioPoseRelocalization::GetInstance().RunUnitTest();
  return 0;
}