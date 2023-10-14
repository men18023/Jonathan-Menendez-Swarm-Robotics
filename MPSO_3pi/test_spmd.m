c = parcluster;
j = createJob(c);
for i = 1:10
    createTask(j,@rand,1,{10});
end
submit(j);
wait(j);
out = fetchOutputs(j);
disp(out{3})