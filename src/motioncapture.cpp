#include "libmotioncapture/motioncapture.h"

namespace libmotioncapture {

  void MotionCapture::getObjectByName(
      const std::string& name,
      Object& result) const
  {
    std::vector<Object> objects;
    getObjects(objects);
    for(const auto& object : objects) {
      if (object.name() == name) {
        result = object;
        return;
      }
    }
    throw std::runtime_error("Object not found!");
  }

}
