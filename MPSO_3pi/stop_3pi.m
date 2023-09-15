function [] = stop_3pi(first_agent,last_agent,agents)
for k = first_agent:last_agent
    robotat_3pi_force_stop(agents{k})
end
end

