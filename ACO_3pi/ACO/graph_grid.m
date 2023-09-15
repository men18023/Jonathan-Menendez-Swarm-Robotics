% Mundo positivo a partir de 1
% Funciona con cuadrados rectangulares (nxm)
% parámetros:
% gridx y gridy son números enteros mayores a 1. Se genera un grafo de gridx x gridy
% La salida es el grafo para implementar el ACO.
function [grafo] = graph_grid(gridx, gridy)

diag_cost = 0.5;
x_lim_pos = gridx;
y_lim_pos = gridy;
x_lim_neg = 1;
y_lim_neg = 1;

grid_x = x_lim_pos - x_lim_neg + 1;
grid_y = y_lim_pos - y_lim_neg + 1;
n = grid_x * grid_y;
[X1, Y1] = meshgrid(1:grid_x, 1:grid_y);

Coords = [reshape(Y1, [n, 1]), reshape(X1, [n, 1])];
simple_move = [1 0; 0 1; -1 0; 0 -1];
diag_move = [-1 1; 1 1; -1 -1; 1 -1];

Name = string((1:n)');

EndNodes = [];
Weight = [];
Eta = [];

for nodo = 1:n
    moved_node = repmat(Coords(nodo, :), [4, 1]) + simple_move;
    diag_moved_node = repmat(Coords(nodo, :), [4, 1]) + diag_move;
    
    % Adjust for rectangular grid boundaries
    vecino_simple = moved_node(moved_node(:, 1) <= x_lim_pos & moved_node(:, 1) >= x_lim_neg & moved_node(:, 2) <= y_lim_pos & moved_node(:, 2) >= y_lim_neg, :);
    vecino_diag = diag_moved_node(diag_moved_node(:, 1) <= x_lim_pos & diag_moved_node(:, 1) >= x_lim_neg & diag_moved_node(:, 2) <= y_lim_pos & diag_moved_node(:, 2) >= y_lim_neg, :);
    
    % Calculate EndNodes based on gridx and gridy
    grid_offset = (nodo - 1) - floor((nodo - 1) / grid_x) * grid_x;
    EndNodes = [EndNodes; grid_y * (vecino_simple(:, 2) - 1) + vecino_simple(:, 1), ones(size(vecino_simple, 1), 1) * nodo];
    Weight = [Weight; ones(size(vecino_simple, 1), 1)];
    Eta = [Eta; ones(size(vecino_simple, 1), 1)];
    
    EndNodes = [EndNodes; grid_y * (vecino_diag(:, 2) - 1) + vecino_diag(:, 1), ones(size(vecino_diag, 1), 1) * nodo];
    Weight = [Weight; ones(size(vecino_diag, 1), 1)];
    Eta = [Eta; ones(size(vecino_diag, 1), 1) * 1 / diag_cost];
end

X = Coords(:, 1);
Y = Coords(:, 2);
G = graph(table(EndNodes, Weight, Eta), table(Name, X, Y));
grafo = simplify(G);

end
