% Comparación entre feval y parfeval
% Luis Alberto Rivera

% Entradas:  N - número de llamados a la función (rank de magic, en este ejemplo)
%           op - opción: 0 - con feval
%                        1 - con parfeval
function Results = feval_vs_parfeval(N, op)
tic;
% Results = cell(1, N);
Results = zeros(N, 1);

if op == 0
    for idx = 1:N
        Results(idx) = feval(@printRandoms,1,10,2);
    end
else
    p = gcp();
    % To request multiple evaluations, use a loop.
    for idx = 1:N
        f(idx) = parfeval(p, @printRandoms, 1,1,10,2);
    end
    
    % Collect the results as they become available.
    for idx = 1:N
        % fetchNext blocks until next results are available.
        [completedIdx, value] = fetchNext(f);
        Results(completedIdx) = value;
    end
end

toc;

end
