% Comparaci�n entre for y parfor
% Luis Alberto Rivera

% Entradas:  N - dimensi�n de la matriz cuadrada aleatoria que se generar�
%           op - opci�n: 0 - sin parfor
%                        1 - con parfor
function a = for_vs_parfor(N, op)
tic
a = zeros(200,1);
if op == 0
    for i = 1:200
        a(i) = max(abs(eig(rand(N))));
    end
else
%     ticBytes(gcp);
    parfor i = 1:200
        a(i) = max(abs(eig(rand(N))));
    end
%     tocBytes(gcp)
end
toc

end