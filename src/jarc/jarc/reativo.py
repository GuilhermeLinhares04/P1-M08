import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd

class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('reactive_navigator')
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.path_history = []  # Store movement history
        self.last_direction = None  # Store last successful direction
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço de movimentação...')
        self.get_logger().info('Conectado ao serviço de movimentação!')

    def move_robot(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            if direction != 'back':  # Don't store backtracking moves
                self.path_history.append(direction)
                self.last_direction = direction
            return future.result()
        else:
            self.get_logger().warning('Movimento não realizado com sucesso.')
            return None

    def get_opposite_direction(self, direction):
        opposites = {'up': 'down', 'down': 'up', 'left': 'right', 'right': 'left'}
        return opposites.get(direction)

    def backtrack(self):
        if self.path_history:
            last_move = self.path_history.pop()
            opposite_direction = self.get_opposite_direction(last_move)
            self.move_robot(opposite_direction)
            return True
        return False

    def navigate_to_target(self):
        while rclpy.ok():
            # Get current state
            if self.last_direction:
                move_result = self.move_robot(self.last_direction)
            else:
                move_result = self.move_robot("down")  # Initial movement

            if move_result is None:
                if not self.backtrack():
                    self.get_logger().warning("Não é possível retroceder mais!")
                    break
                continue

            # Check if target is reached
            if move_result.left == 't' or move_result.right == 't' or \
               move_result.up == 't' or move_result.down == 't':
                self.get_logger().info('Alvo alcançado!')
                break

            # Toma decisões de navegação baseadas nas leituras dos sensores
            if move_result.down == 'f' and self.last_direction != 'up':
                self.last_direction = "down"
            elif move_result.right == 'f' and self.last_direction != 'left':
                self.last_direction = "right"
            elif move_result.left == 'f' and self.last_direction != 'right':
                self.last_direction = "left"
            elif move_result.up == 'f' and self.last_direction != 'down':
                self.last_direction = "up"
            else:
                if not self.backtrack():
                    self.get_logger().warning("Robô bloqueado por obstáculos em todas as direções!")
                    break

def main(args=None):
    rclpy.init(args=args)
    navigator = ReactiveNavigator()
    navigator.navigate_to_target()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
