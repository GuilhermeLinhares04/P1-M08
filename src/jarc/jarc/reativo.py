import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd  # Interface do serviço para o comando de movimento

class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('reactive_navigator')
        self.client = self.create_client(MoveCmd, '/move_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço /move_command...')
        
        self.robot_position = None
        self.target_position = None
    
    def send_move_request(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response:
            self.robot_position = response.robot_pos
            self.target_position = response.target_pos
            return response
        else:
            self.get_logger().error("Falha ao mover o robô.")
            return None

    def navigate(self):
        while self.robot_position != self.target_position:
            response = self.send_move_request("down")  # Exemplo de direção inicial
            if not response or not response.success:
                break
            
            # Decidir a direção com base nos dados dos sensores
            directions = {
                'left': response.left,
                'down': response.down,
                'up': response.up,
                'right': response.right
            }
            
            for direction, status in directions.items():
                if status == 't':  # Encontrou o alvo
                    self.get_logger().info(f'Alvo encontrado na direção {direction}')
                    self.send_move_request(direction)
                    return True
                elif status == 'f':  # Espaço livre encontrado
                    self.get_logger().info(f'Movendo para {direction}')
                    self.send_move_request(direction)
                    break

            # Pausa breve para evitar loop rápido
            rclpy.sleep(0.5)

        self.get_logger().info("O robô alcançou o destino.")

def main(args=None):
    rclpy.init(args=args)
    navigator = ReactiveNavigator()
    navigator.navigate()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
