import time
from typing import List
import rclpy
from rclpy.node import Node, Client
from rclpy import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int8

from festo_edcon_interface.srv import EdconCommand

import paho.mqtt.client as mqtt
import threading
import json
from asyncio import Future
import time


class DashboardCommander(Node):
    is_done = False
    mqtt_config = {
        "host": "localhost",
        "port": 1883,
        "command_topic": "edconMqttCommand",
        "jog_topic": "edconMqttJog",
        "result_topic": "edconMqttResult",
    }
    mqttc_thread = None
    ros_cli: Client = None
    jog_publisher = None

    def __init__(self):
        super().__init__("festo_edcon_mqtt")
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_disconnect = self.on_disconnect
        self.mqttc.on_message = self.on_message
        self.mqttc.on_subscribe = self.on_subscribe
        self.mqttc.on_unsubscribe = self.on_unsubscribe
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.declare_parameter("mqtt_config.host", "localhost")
        self.declare_parameter("mqtt_config.port", 1883)
        self.declare_parameter("mqtt_config.command_topic", "edconMqttCommand")
        self.declare_parameter("mqtt_config.jog_topic", "edconMqttJog")
        self.declare_parameter("mqtt_config.result_topic", "edconMqttResult")

        ns = self.get_namespace()
        ns = ns if ns != "/" else ""

        self.ros_cli = self.create_client(EdconCommand, ns + "/festo_edcon_cmd")
        while not self.ros_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        self.jog_publisher = self.create_publisher(Int8, ns + "/festo_edcon_jog", 11)

        self.mqttc_thread = threading.Thread(target=self.mqtt_loop)
        self.mqttc_thread.start()

    def on_set_parameters(self, params: List[Parameter]):
        for param in params:
            config_name = param.name.replace("mqtt_config.", "")
            if (
                config_name in self.mqtt_config
                and self.mqtt_config[config_name] != param.value
            ):
                self.get_logger().info(
                    f"config {config_name} is changed from {self.mqtt_config[config_name]} to {param.value}"
                )
                self.mqtt_config[config_name] = param.value
                self.mqttc.disconnect()

        return SetParametersResult(successful=True)

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
        if reason_code_list[0].is_failure:
            self.get_logger().error(
                f"Broker rejected you subscription: {reason_code_list[0]}"
            )
        else:
            self.get_logger().info(
                f"Broker granted the following QoS: {reason_code_list[0].value}"
            )

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            self.get_logger().debug(
                "unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)"
            )
        else:
            self.get_logger().warn(
                f"Broker replied with failure: {reason_code_list[0]}"
            )
        client.disconnect()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            self.get_logger().error(
                f"Failed to connect: {reason_code}. loop_forever() will retry connection"
            )
        else:
            self.get_logger().debug(f"mqttc connected")
            client.subscribe(self.mqtt_config["command_topic"])
            client.subscribe(self.mqtt_config["jog_topic"])

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.get_logger().debug("disconnect mqttc")

    def on_message(self, client, userdata, msg):
        self.get_logger().info(msg.topic + " " + str(msg.payload))
        try:
            if msg.topic == self.mqtt_config["command_topic"]:
                json_dict = json.loads(msg.payload.decode("utf-8"))
                command = json_dict["command"]
                data = json.dumps(json_dict["data"]) if "data" in json_dict else ""
                future = self.send_request(command, data)

                def done(future: Future):
                    res = future.result()
                    res_dict = {}

                    for key in res._fields_and_field_types.keys():
                        attr = getattr(res, key)
                        attr_type = type(attr)
                        if (
                            attr_type is str
                            or attr_type is float
                            or attr_type is int
                            or attr_type is bool
                        ):
                            res_dict[key] = attr

                    self.get_logger().info(str(future.result()))
                    self.mqttc.publish(
                        self.mqtt_config["result_topic"], json.dumps(res_dict)
                    )

                future.add_done_callback(done)
            elif msg.topic == self.mqtt_config["jog_topic"]:
                pub_msg = Int8()
                pub_msg.data = int(msg.payload)
                self.jog_publisher.publish(pub_msg)

        except Exception as ex:
            self.get_logger().error(str(ex))

    def send_request(self, cmd, data) -> Future:
        req = EdconCommand.Request()
        req.command = cmd
        req.data = data
        return self.ros_cli.call_async(req)

    def mqtt_loop(self):
        while self.is_done == False:
            try:
                host = self.mqtt_config["host"]
                port = self.mqtt_config["port"]
                self.get_logger().info(f"connect mqttc {host}:{port}")
                self.mqttc.connect(host, port)
                self.mqttc.loop_forever()
                self.get_logger().info(f"disconnected mqttc {host}:{port}")
            except Exception as ex:
                self.get_logger().error(str(ex))
                pass
            time.sleep(1)


def main():
    rclpy.init()
    dshbd_commander = DashboardCommander()
    rclpy.spin(dshbd_commander)
    dshbd_commander.is_done = True
    dshbd_commander.mqttc.disconnect()
    dshbd_commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
