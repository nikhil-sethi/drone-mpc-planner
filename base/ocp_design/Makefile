.PHONY: intercept_in_planes tti min_nlp
.SILENT: intercept_in_planes min_nlp

all: min_nlp tti intercept_in_planes

intercept_in_planes:
	cd intercept_in_planes && python3 ocp_intercept_in_planes.py
	echo "Intercept_in_planes_Optimizer ready to be build in executor."

tti:
	cd tti && python3 ocp_tti.py
	echo "TTI_Optimizer ready to be build in executor."

min_nlp:
	cd min_nlp && python3 ocp_min_nlp.py
	echo "Min_Nlp_Optimizer ready to be build in executor."
