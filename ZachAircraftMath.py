#Team 4 Aircraft Math

import numpy as np


"""
Args:
alt: Altitude [m]
temp_offset: [NOT SET UP] Temperature offset from the ground level temp of 15C [C or K]

Returns: Air density at specified altitude and temperature offset [kg/m^3]
Curve fitted from https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
"""
def altitudeToDensity(alt=0, temp_offset=0):
    sealevel = 1.225 #Sea leavel density
    ratio = (-3E-14 * alt**3) + (3E-9 * alt**2) + (-9E-5 * alt) + 0.9972 #Air desnity/Sea level density
    return ratio*sealevel
#!altitudeToDensity



"""
Args:
AR: Aspect Ratio

Returns: Oswald Efficiency e, assuming straight wings
From eqn 12.48 of Raymer
"""
def oswaldEfficiency(AR):
	return 1.78*(1-0.045*(AR**0.68)) - 0.64
#!oswaldEfficiency



"""
Args:
AR: Aspect Ratio

Returns: Lift induced drag constant k
From Raymer eqn 12.47
"""
def dragFactor(AR):
	e = oswaldEfficiency(AR)
	return 1/(np.pi*AR*e)
#!dragFactor



"""
Args:
WS: Wing loading [lb/ft^2]
alt: Altitude [m]
c_Dmin: Minimum drag coefficient
c_L0: Lift coefficient
AR: Aspect ratio

Returns: Velocity fro best L/D ratio [m/s]
"""
def findVelocityLD(WS, alt, c_Dmin, c_L0, AR):
    WS_metric = WS*4.88243*9.81 #Gets WS in N/m^2
    rho = altitudeToDensity(alt)
    k = dragFactor(AR)

    var1 = (WS_metric*2/rho)**2
    var2 = k/(c_Dmin + k*c_L0**2)
    return (var1*var2)**0.25
#!findVelocityLD



"""
Args:
WS: Wing loading [lb/ft^2]
alt: Altitude [m]
c_Dmin: Minimum drag coefficient
c_L0: Lift coefficient
AR: Aspect ratio
LD: Lift to drag ratio

Return: Velocty for best rate of climb [m/s]
"""
def findVelocityRoC(WS, alt, c_Dmin, c_L0, AR, LD):
    WS_metric = WS*4.88243*9.81 #Gets WS in N/m^2
    rho = altitudeToDensity(alt)
    k = dragFactor(AR)

    var1 = WS_metric*(2*k*LD*c_L0 + 1)
    var2 =  LD*rho*(c_Dmin + k*c_L0**2)

    H = var1/var2

    v_ld = findVelocityLD(WS, rho, c_Dmin, c_L0, AR)

    return (H - (H**2 - v_ld**4)**0.5)**0.5
#!findVelocityRoC



"""
Args:
alt: Altitude [m]
v: Velocity [m/s]

Return: Dynamic pressure [Pa (N/m^2)]
"""
def dynamicPressure(alt, v):
	rho = altitudeToDensity(alt)
	return 0.5*rho*v*v
#!dynamicPressure



"""
Args:
WS: Wing loading [lb/ft^2]
AR: Aspect ratio
C_Dmin: Minimum drag coefficient
alt: ALtitude [m]
v: Speed [m/s]

Return: Lift to drag ratio
"""
def liftDragRatio(WS, AR, C_Dmin, alt, v):
	q_metric = dynamicPressure(alt, v)
	q_imp = 0.02089*q_metric
	k = dragFactor(AR)

	LD_inv = (q_imp*C_Dmin/WS) + (WS*k/q_imp)
	return (1/LD_inv)
#!LiftDrag



"""
Series of SFC and BSFC converters
BSFC [g/kW/h]
SFC [lb/hp/h]
InvMeters [1/m]
"""
def BSFCtoSFC(BSFC):
	return BSFC*1.64399E-3

def SFCtoInvMeters(SFC):
	return SFC*1.65699E-6

def BSFCtoInvMeters(BSFC):
	return SFCtoInvMeters(BSFCtoSFC(BSFC))
#!SFC Converters

def kg_to_lb(mass):
	return mass*2.20462

def lb_to_kg(mass):
	return mass*0.453592



"""
Takes: fuel weight
Returns C02 emissions
"""
def fuelToCO2(fuel_weight):
	return 3.01*fuel_weight