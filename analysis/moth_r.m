%system props:
FOV = 90
max_depth=10
a=(FOV/360)*pi*max_depth^2

%10 systems per ha:
A=100*100
k=10
P_hit = 0.1

%moth props:
flight_window = 3 %hours
flight_periode = 5 % minutes
flight_dist = 20 %meter
expected_lifetime = 14 %days

r_moth = 1.14
catts_per_moth = 300 % per cycle
moth_cycle = 28 % days
moth_in_flux = 10 * moth_cycle
moth_out_flux = 1 * moth_cycle

p_not_seen_1 = 1-a/A % chance of a randomly placed moth NOT being seen by 1 system in an hectare
p_not_seen_all = p_not_seen_1^k
p_seen = 1- p_not_seen_all % chance of a moth being seen by k randomly placed systems (with possible overlap)

a_eff = p_seen * A % effective total hunt area
a_prime = a_eff/k % effective hunt area per system

n_moth_flights_per_night = flight_window * (60/flight_periode)
dx_per_night = n_moth_flights_per_night * flight_dist % meter

line_dist = sqrt(A) / (dx_per_night / sqrt(A))
n_encounters_per_night = sqrt(a_eff) / line_dist


E_moth_lifetime = (1 / P_hit ) / n_encounters_per_night

%linear birth and death process
n = 0
for i = 0:9
	n = (moth_in_flux+n*catts_per_moth) - (moth_out_flux+n+n*n_encounters_per_night*moth_cycle*P_hit);
    disp (['After ' num2str(i) ' cycles, we have ' num2str(n) ' moths'])
end






