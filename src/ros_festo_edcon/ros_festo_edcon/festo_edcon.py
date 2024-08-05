import json
import time
from typing import List
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, Parameter

from edcon.edrive.com_modbus import ComModbus
from edcon.edrive.motion_handler import MotionHandler
from edcon.utils.logging import Logging
from festo_edcon_interface.srv import EdconCommand


class FestoEdcon(Node):
    edcon_config = {"host": "127.0.0.1"}
    modbus: ComModbus = None
    mot_handler: MotionHandler = None

    def __init__(self):
        super().__init__("festo_edcon")
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.declare_parameter("edcon_config.host", "localhost")

        Logging()
        self.modbus = ComModbus(self.edcon_config["host"])
        self.mot_handler = MotionHandler(self.modbus)

        self.srv = self.create_service(
            EdconCommand, "festo_edcon_cmd", self.execute_command
        )

    def on_set_parameters(self, params: List[Parameter]):
        for param in params:
            config_name = param.name.replace("edcon_config.", "")
            if (
                config_name in self.edcon_config
                and self.edcon_config[config_name] != param.value
            ):
                self.get_logger().info(
                    f"config {config_name} is changed from {self.edcon_config[config_name]} to {param.value}"
                )
                self.edcon_config[config_name] = param.value

                if config_name == "host":
                    if self.modbus != None:
                        self.modbus.shutdown()
                    self.modbus = ComModbus(self.edcon_config["host"])

        return SetParametersResult(successful=True)

    def execute_command(self, request, response):
        response.success = False
        response.answer = "command not found"

        try:
            if request.command == "get_status":
                response.answer = self.get_status()
                response.success = True
            elif request.command == "ack":
                response.answer = "acknowledge_faults"
                response.success = self.acknowledge_faults()
            elif request.command == "set_position":
                req_dict = json.loads(request.data)
                response.sucess = self.set_position(
                    position=int(req_dict["position"]) if "position" in req_dict else 0,
                    velocity=int(req_dict["velocity"]) if "velocity" in req_dict else 0,
                    absolute=(
                        bool(req_dict["absolute"]) if "absolute" in req_dict else False
                    ),
                )
                response.answer = self.get_status()
            elif request.command == "run_record_task":
                req_dict = json.loads(request.data)
                record_number = (
                    int(req_dict["record_number"]) if "record_number" in req_dict else 1
                )
                response.sucess = self.run_record_task(
                    record_number=record_number if record_number > 0 else 1,
                )
                response.answer = self.get_status()
            elif request.command == "stop_motion":
                response.answer = "stop_motion"
                response.success = self.stop_motion()

        except Exception as ex:
            self.get_logger().error(ex)

        return response

    def get_status(self):
        status = {
            "current_position": 0,
            "current_velocity": 0,
            "fault_present": False,
            "fault_string": "Fault message string",
            "ready_for_motion": False,
        }

        status["current_position"] = int(self.mot_handler.current_position())
        status["current_velocity"] = int(self.mot_handler.current_velocity())
        status["fault_present"] = self.mot_handler.fault_present()
        status["fault_string"] = str(self.mot_handler.fault_string())
        status["ready_for_motion"] = self.mot_handler.ready_for_motion()

        return json.dumps(status)

    def acknowledge_faults(self):
        return self.mot_handler.acknowledge_faults()

    def jog(self):
        pass

    def set_position(self, position, velocity, absolute):
        if self.mot_handler.fault_present():
            self.mot_handler.acknowledge_faults()

        self.mot_handler.enable_powerstage()

        if not self.mot_handler.referenced():
            self.mot_handler.referencing_task()

        return self.mot_handler.position_task(
            position=position,
            velocity=velocity,
            absolute=absolute,
            nonblocking=False,
        )

    def stop_motion(self):
        self.mot_handler.stop_motion_task()
        time.sleep(0.1)
        return self.mot_handler.stopped()

    def run_record_task(self, record_number):
        if self.mot_handler.fault_present():
            self.mot_handler.acknowledge_faults()
        self.mot_handler.enable_powerstage()
        result = self.mot_handler.record_task(record_number, False)
        return result

def __exit__(self, ):
    self.mot_handler()

def main():
    rclpy.init()
    edcon = FestoEdcon()
    rclpy.spin(edcon)
    edcon.mot_handler.shutdown()
    edcon.modbus.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()