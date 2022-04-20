#include "OdometryHolo.h"
#include "encoders.h"
#include "utils.h"
#include "communication.h"
#include "BytesWriteBuffer.h"
#include "messages.h"


void OdometryHolo::init() {
  initEnc1(true);
  initEnc2(false);
  initEnc3(false);
  initEnc4(true);

  auto cb_recalage = [this](Message& msg) {
      if(msg.has_pos() && msg.msg_type() == Message::MsgType::COMMAND) {
        double x = msg.pos().get_x();
        double y = msg.pos().get_y();
        double theta = msg.pos().get_theta();
        //set_pos(x, y, theta);
    }
  };
  register_callback(cb_recalage);
}
