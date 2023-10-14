%q = parallel.pool.PollableDataQueue;
Q_Agents = 1000;
val = 0;
fitness_local = zeros(Q_Agents,1);
fitness_global = zeros(Q_Agents,1);
if val ==0
    tic
    parfor i = 1:Q_Agents
        fitness_local(i) = fitness(-2.5 +5*rand(),-2.5+5*rand(), 1);
        fitness_global(i) = fitness(-2.5 +5*rand(),-2.5+5*rand(), 1);
    end
    toc
else
    tic
    for i = 1:Q_Agents
        fitness_local(i) = fitness(-2.5 +5*rand(),-2.5+5*rand(), 1);
        fitness_global(i) = fitness(-2.5 +5*rand(),-2.5+5*rand(), 1);
    end
    toc
end
%[data, OK] = poll(q,0.2);
