import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd

class WallFollowingSolver(Node):
    def __init__(self):
        super().__init__('wall_following_solver')
        self.cli = self.create_client(MoveCmd, 'move_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço move command...')
        self.req = MoveCmd.Request()
        self.future = None
        self.solved = False
        self.visited_positions = set()
        self.stack = []
        self.direction_order = ['right', 'down', 'left', 'up']  # Prioridade: direita -> frente -> esquerda -> trás
        self.timer = self.create_timer(0.05, self.timer_callback)

    # Ficar chamando o serviço de movimentação constantemente
    def timer_callback(self):
        if self.solved:
            return

        if self.future is None or self.future.done():
            if self.future is not None and self.future.result() is not None:
                response = self.future.result()
                self.process_response(response)
            elif self.future is None:
                self.req.direction = 'down'  # Movimento inicial para começar
                self.future = self.cli.call_async(self.req)

    # Determina o que fazer com a resposta do serviço de movimentação
    def process_response(self, response):
        left = response.left
        down = response.down
        up = response.up
        right = response.right
        robot_pos = tuple(response.robot_pos)
        target_pos = tuple(response.target_pos)

        self.get_logger().info(f'Posição do robô: {robot_pos}')
        self.get_logger().info(f'Posição do alvo: {target_pos}')
        self.get_logger().info(f'Movimentos - Esquerda: {left}, Direita: {right}, Cima: {up}, Baixo: {down}')

        # Verifica se o robô chegou ao alvo
        if robot_pos == target_pos:
            self.solved = True
            self.get_logger().info('Alvo encontrado!')
            rclpy.shutdown()
            return

        # Armazena a posição atual na pilha se for nova
        if robot_pos not in self.visited_positions:
            self.stack.append(robot_pos)
        
        self.visited_positions.add(robot_pos)
        self.get_logger().info(f'Posições visitadas: {self.visited_positions}')

        if not response.success:
            self.get_logger().info(f'Movimento {self.req.direction} não funcionou.')
            blocked_pos = self.get_new_position(robot_pos, self.req.direction)
            self.visited_positions.add(blocked_pos)
            self.get_logger().info(f'Posição bloqueada adicionada: {blocked_pos}')

        possible_moves = self.get_possible_moves(left, right, up, down, robot_pos)
        self.get_logger().info(f'Movimentos possíveis: {possible_moves}')

        # Algoritmo de backtracking
        if possible_moves:
            for direction in self.direction_order:
                for move_direction, new_pos in possible_moves:
                    if move_direction == direction:
                        self.req.direction = move_direction
                        self.future = self.cli.call_async(self.req)
                        self.get_logger().info(f'Movendo para {move_direction}')
                        return
        else:
            # Sem movimentos possíveis - fazer backtracking
            self.get_logger().info('Nenhum movimento possível, iniciando backtracking')
            if self.stack:
                last_pos = self.stack.pop()
                self.req.direction = self.get_direction_to(robot_pos, last_pos)
                self.future = self.cli.call_async(self.req)
                self.get_logger().info(f'Fazendo backtracking para {self.req.direction}')
            else:
                self.get_logger().info('Nenhuma posição restante para backtracking, encerrando')
                rclpy.shutdown()

    # Retorna os movimentos possíveis a partir da posição atual
    def get_possible_moves(self, left, right, up, down, robot_pos):
        possible_moves = []
        x, y = robot_pos

        new_pos = (x, y - 1)
        if (left == 'f' or left == 't') and new_pos not in self.visited_positions:
            possible_moves.append(('left', new_pos))
        new_pos = (x, y + 1)
        if (right == 'f' or right == 't') and new_pos not in self.visited_positions:
            possible_moves.append(('right', new_pos))
        new_pos = (x - 1, y)
        if (up == 'f' or up == 't') and new_pos not in self.visited_positions:
            possible_moves.append(('up', new_pos))
        new_pos = (x + 1, y)
        if (down == 'f' or down == 't') and new_pos not in self.visited_positions:
            possible_moves.append(('down', new_pos))

        return possible_moves

    # Atualiza a posição do robô a nova posição a partir da posição atual e da direção
    def get_new_position(self, pos, direction):
        x, y = pos
        if direction == 'left':
            return (x, y - 1)
        elif direction == 'right':
            return (x, y + 1)
        elif direction == 'up':
            return (x - 1, y)
        elif direction == 'down':
            return (x + 1, y)
        else:
            return pos

    # Função do backtracking 
    def get_direction_to(self, from_pos, to_pos):
        """Retorna a direção do 'from_pos' para 'to_pos'."""
        x1, y1 = from_pos
        x2, y2 = to_pos
        
        if x1 == x2:
            if y2 > y1:
                return 'right'
            elif y2 < y1:
                return 'left'
        elif y1 == y2:
            if x2 > x1:
                return 'down'
            elif x2 < x1:
                return 'up'
        
        self.get_logger().error(f'Posições não são adjacentes: de {from_pos} para {to_pos}')
        return 'down'

def main(args=None):
    rclpy.init(args=args)
    wall_following_solver = WallFollowingSolver()
    rclpy.spin(wall_following_solver)
    wall_following_solver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
