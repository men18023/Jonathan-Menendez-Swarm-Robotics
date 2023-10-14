function [num,count]  = printRandoms(lower, upper, count)
  for i = 1:count
    num = rand(1, 1) * (upper - lower + 1) + lower;
    disp(num)
  end
end
% function nums = printRandoms(lower, upper, count)
%     %rng(seed);  % Set the random seed
%     nums = zeros(count, 1);
%     for i = 1:count
%         nums(i) = rand(1, 1) * (upper - lower + 1) + lower;
%     end
% end


