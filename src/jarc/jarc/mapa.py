import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, GetMap
import numpy as np
from queue import PriorityQueue

class MapNavigator(Node):
    def __init__(self):
        super().__init__('map_navigator')
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.map_client = self.create_client(GetMap, '/get_map')

        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço de movimentação...')
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço de mapa...')
        self.get_logger().info('Conectado aos serviços de movimentação e mapa!')

        # Configurações iniciais
        self.occupancy_grid = None
        self.grid_shape = None
        self.path = []

    def get_map(self):
        request = GetMap.Request()
        future = self.map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.occupancy_grid = future.result().occupancy_grid_flattened
            self.grid_shape = future.result().occupancy_grid_shape
            return True
        else:
            self.get_logger().error('Não foi possível obter o mapa.')
            return False

    def reconstruct_grid(self):
        if self.occupancy_grid and self.grid_shape:
            grid_2d = np.array(self.occupancy_grid).reshape(self.grid_shape)
            return grid_2d
        return None

    def a_star_search(self, start, goal, grid):
        rows, cols = grid.shape
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for direction, (dr, dc) in [('down', (1, 0)), ('up', (-1, 0)), ('right', (0, 1)), ('left', (0, -1))]:
                neighbor = (current[0] + dr, current[1] + dc)
                
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 'b':
                    tentative_g_score = g_score[current] + 1

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = (current, direction)
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        open_set.put((f_score[neighbor], neighbor))

        return None

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            current, direction = came_from[current]
            path.append(direction)
        path.reverse()
        return path

    def move_along_path(self):
        for direction in self.path:
            result = self.move_robot(direction)
            if not result or not result.success:
                self.get_logger().error('Falha ao mover o robô.')
                break
            if (result.robot_pos[0], result.robot_pos[1]) == (result.target_pos[0], result.target_pos[1]):
                self.get_logger().info('Alvo alcançado com sucesso!')
                break

    def move_robot(self, direction):
        request = MoveCmd.Request()
        request.direction = direction
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def navigate_to_target(self):
        if not self.get_map():
            return

        grid = self.reconstruct_grid()
        start = tuple(np.argwhere(np.array(grid) == 'r')[0])  # posição inicial do robô
        goal = tuple(np.argwhere(np.array(grid) == 't')[0])   # posição do alvo

        self.path = self.a_star_search(start, goal, grid)
        if self.path:
            self.move_along_path()
        else:
            self.get_logger().error('Nenhum caminho encontrado para o alvo.')

def main(args=None):
    rclpy.init(args=args)
    navigator = MapNavigator()
    navigator.navigate_to_target()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
