function [fitness] = fitness(x, y, BENCHMARK_TYPE)
  if BENCHMARK_TYPE == 0
    fitness = x^2 + y^2;
  elseif BENCHMARK_TYPE == 1
    fitness = (1 - x)^2 + 100 * (y - x^2)^2;
  elseif BENCHMARK_TYPE == 2
    fitness = (x + 2*y - 7)^2 + (2*x + y - 5)^2;
  elseif BENCHMARK_TYPE == 3
    fitness = (x^2 + y - 11)^2 + (x + y^2 - 7)^2;
  elseif BENCHMARK_TYPE == 4
    fitness = 0.5 + (cos(sin(sqrt(x^2 - y^2))))^2 - 0.5 / (1 + 0.001*(x^2 + y^2))^2;
  elseif BENCHMARK_TYPE == 5
    fitness = -(sin(2*x - y)^2 * sin(2*x + y)^2) / sqrt(x^2 + y^2);
  end
end
