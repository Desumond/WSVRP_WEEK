from django.views.decorators.csrf import csrf_exempt

from spyne.server.django import DjangoApplication
from spyne.model.primitive import Unicode, Integer, AnyDict
from spyne.service import ServiceBase
from spyne.service import Service
from spyne.server.wsgi import WsgiApplication
from spyne.protocol.json import JsonDocument
from spyne.protocol.xml import XmlDocument
from spyne.protocol.soap import Soap11

from spyne.application import Application
from spyne.decorator import rpc

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import requests, polyline, json
from datetime import datetime, timedelta



#Coefficients for taking to account the impact of traffic
traffic_coefficient = [1.036,1.015,1.004,1,1.003,1.018,1.066,1.190,1.254,1.244,1.221,1.218,1.218,1.222,1.221,1.216,1.226,1.228,1.241,1.232,1.210,1.171,1.130,1.076]

server = 'http://3.19.181.200:5000'
#server = 'http://192.168.100.32:5000'

# Query module
class MakeModeling(Service):
	@rpc(AnyDict, AnyDict, Integer, Integer, Integer, AnyDict, Integer, Integer, AnyDict, _returns=AnyDict)
	def vrp_modeling(ctx, school, bus_types, max_route_time, direction, time, school_time, service_time, modeling_time, points):
		solution = {}
		solution['info'] = {}
		gotomodeling = 0
		er = 0
		error = ''
		
		#Check the correctness of data for modeling
		try:
			data_input = {'school': school,
						  'bus_types': bus_types,
						  'max_route_time': max_route_time,
						  'direction': direction,
						  'time': time,
						  'school_time': school_time,
						  'service_time': service_time,
						  'modeling_time': modeling_time,
						  'points': points}

			# Create data for modeling routes for school or business
			if school['form'] == 0 or school['form'] == 1:
				data = create_data_model(data_input)
			# Create data for modeling route for business with defined start and finish
			elif school['form'] == 2:
				data = create_data_model_route(data_input)
			# Create data for modeling route for business with defined all points
			elif school['form'] == 3:
				data = create_data_model_route_defined(data_input)
			else:
				data = {}
				solution = {'status': {'code': 'TypeError',
									   'message': 'Solicitud no válida. Uno de los elementos de datos de solicitud no existe o no coincide con el patrón. Verifique que la solicitud sea correcta.'}}
				data['error'] = 'TypeError'

			# If the data matches the template, then go to the modeling
			if data['error'] == 'NO':
				gotomodeling = 1

			# If the data is correct, but modeling is not possible, then we raise other errors
			elif data['error'] == 'TravelTimeError':
				solution['status'] = {'code': data['error'], 'message': data['info']}

			elif data['error'] == 'CapacityError':
				solution['status'] = {'code': data['error'], 'message': data['info']}

			elif data['error'] == 'Error de conexión! El servidor OSRM no responde, no es posible obtener la matriz de tiempo.':
				solution['status'] = {'code': 'ConnectionError',
									  'message': 'Error de conexión! El servidor OSRM no responde, no es posible obtener la matriz de tiempo.'}

			elif data['error'] == 'TypeError':
				solution['status'] = {'code': 'TypeError',
									  'message': 'Solicitud no válida. Uno de los elementos de datos de solicitud no existe o no coincide con el patrón. Verifique que la solicitud sea correcta.'}
			else:
				solution = {'status': {'code': 'UnknownDataERROR', 'message': 'Error desconocido con datos de entrada'}}

		# If the data is incorrect, then we raise various types of errors
		except TypeError:
			solution = {'status': {'code': 'TypeError',
									   'message': 'Solicitud no válida. Uno de los elementos de datos de solicitud no existe o no coincide con el patrón. Verifique que la solicitud sea correcta.'}}
		except LookupError:
			solution = {'status': {'code': 'LookupError',
								   'message': 'Solicitud no válida. Uno de los elementos de datos de solicitud no existe o no coincide con el patrón. Verifique que la solicitud sea correcta.'}}
		except:
			# For other data errors
			solution = {'status': {'code': 'UnknownDataERROR', 'message': 'Error desconocido con datos de entrada'}}

		# Call the solution function
		if gotomodeling == 1:

			# Modeling for school (1 modeling for all points and transpose to all days)
			if school['form'] == 0:
				solver_result = main(data, 'all')

				# If a solution was found, then output
				if solver_result['status'] == 1:
					solution = print_solution(data, solver_result, data_input)
					solution['status'] = {'code': 'OK', 'message': 'Problema resuelto con éxito'}
				# If not, then we raise various errors related to the solver
				elif solver_result['status'] == 2:
					solution = {'status': {'code': 'NoSolutionERROR',
										   'message': 'No se encontró solución al problema. Verifique que los datos sean correctos o intente aumentar la capacidad'}}
				elif solver_result['status'] == 3:
					solution = {'status': {'code': 'TimeERROR',
										   'message': 'No se encontró solución al problema. Límite de tiempo alcanzado antes de encontrar una solución. Verifique que los datos sean correctos o intente aumentar la capacidad'}}
				elif solver_result['status'] == 4:
					solution = {'status': {'code': 'ModelERROR',
										   'message': 'No se encontró solución al problema. Verifique que los datos sean correctos'}}
				else:
					solution = {'status': {'code': 'UnknownSolverERROR', 'message': 'Error desconocido con solucionador'}}

			# Modeling for business (modeling for every day)
			elif school['form'] == 1 or school['form'] == 2:
				local_solution = []

				# For days from list data['school_time']
				for key in data['school_time'].keys():
					solver_result = main(data, key)

					# If a solution was found, then output
					if solver_result['status'] == 1:

						# If defined only start or finish:
						if school['form'] != 2:
							local_solution.append({key: print_solution_empresa(data, solver_result, data_input, key)})

						# If defined start and finish:
						else:
							local_solution.append({key: print_solution_empresa_route(data, solver_result, data_input, key)})

					# If not, then raise various errors related to the solver
					elif solver_result['status'] == 2:
						error = {'status': {'code': 'NoSolutionERROR',
											'message': 'No se encontró solución al problema. Verifique que los datos sean correctos o intente aumentar la capacidad',
											'day': key}}
						er = 1
					elif solver_result['status'] == 3:
						error = {'status': {'code': 'TimeERROR',
											'message': 'No se encontró solución al problema. Límite de tiempo alcanzado antes de encontrar una solución. Verifique que los datos sean correctos o intente aumentar la capacidad',
											'day': key}}
						er = 1
					elif solver_result['status'] == 4:
						error = {'status': {'code': 'ModelERROR',
											'message': 'No se encontró solución al problema. Verifique que los datos sean correctos',
											'day': key}}
						er = 1
					else:
						error = {'status': {'code': 'UnknownSolverERROR', 'message': 'Error desconocido con solucionador', 'day': key}}
						er = 1

				# If there is no solution for at least one day:
				if er == 1:
					solution = error

				# If solution exists, print solution
				else:
					solution['status'] = {'code': 'OK', 'message': 'Problema resuelto con éxito'}
					if data['direction'] == 0:
						solution['info']['direction'] = 'Recogida'
					else:
						solution['info']['direction'] = 'Entrega'
					solution['days'] = local_solution

			# Build a route for a defined order of points.
			elif school['form'] == 3:
				local_solution = []

				# For days from list data['school_time'] print solution
				for key in data['school_time'].keys():
					solution_loc = print_solution_empresa_route_defined(data, key)
					local_solution.append({key: solution_loc['result']})

				solution['status'] = {'code': 'OK', 'message': 'Problema resuelto con éxito'}
				solution['info'] = {}
				if data['direction'] == 0:
					solution['info']['direction'] = 'Recogida'
				else:
					solution['info']['direction'] = 'Entrega'

				# Add warnings if the maximum time is exceeded
				solution['warning'] = solution_loc['warning']
				solution['days'] = local_solution
		else:
			pass

		return solution


# Solver
def main(data, key):
	# Create the routing index manager.
	manager = pywrapcp.RoutingIndexManager(len(data['time_matrix'][key]), data['num_vehicles'], data['starts'],
										   data['ends'])

	# Create Routing Model.
	routing = pywrapcp.RoutingModel(manager)

	# Create and register a transit callback.
	def distance_callback(from_index, to_index):
		# Returns the distance between the two nodes
		# Convert from routing variable Index to distance matrix NodeIndex.
		from_node = manager.IndexToNode(from_index)
		to_node = manager.IndexToNode(to_index)
		return data['time_matrix'][key][from_node][to_node]

	transit_callback_index = routing.RegisterTransitCallback(distance_callback)

	# Define cost of each arc.
	routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

	# Add Capacity constraint.
	def demand_callback(from_index):
		"""Returns the demand of the node."""
		# Convert from routing variable Index to demands NodeIndex.
		from_node = manager.IndexToNode(from_index)
		return data['demands'][key][from_node]

	demand_callback_index = routing.RegisterUnaryTransitCallback(
		demand_callback)
	routing.AddDimensionWithVehicleCapacity(
		demand_callback_index,
		0,  # null capacity slack
		data['vehicle_capacities'],  # vehicle maximum capacities
		True,  # start cumul to zero
		'Capacity')

	# Add Time Windows constraint.
	time = 'Time'
	routing.AddDimension(
		transit_callback_index,
		6000,  # allow waiting time
		data['max_route_time'],  # maximum time per vehicle
		False,  # Don't force start cumul to zero.
		time)
	time_dimension = routing.GetDimensionOrDie(time)
	# Add time window constraints for each location except depot.
	for location_idx, time_window in enumerate(data['time_windows'][key]):
		if location_idx == 0:
			continue
		index = manager.NodeToIndex(location_idx)
		time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
	# Add time window constraints for each vehicle start node.
	for vehicle_id in range(data['num_vehicles']):
		index = routing.Start(vehicle_id)
		time_dimension.CumulVar(index).SetRange(data['time_windows'][key][0][0], data['time_windows'][key][0][1])

	# Instantiate route start and end times to produce feasible times.
	for i in range(data['num_vehicles']):
		routing.AddVariableMinimizedByFinalizer(
			time_dimension.CumulVar(routing.Start(i)))
		routing.AddVariableMinimizedByFinalizer(
			time_dimension.CumulVar(routing.End(i)))
		routing.SetFixedCostOfVehicle(data['vehicle_price'][i], i)  # Create penalty for using extra bus

	# Setting first solution heuristic.
	search_parameters = pywrapcp.DefaultRoutingSearchParameters()
	search_parameters.time_limit.seconds = data['modeling_time']
	search_parameters.first_solution_strategy = (
		routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)  # method - PATH_CHEAPEST_ARC

	# Adding local search method for escape from local extremal points
	search_parameters.local_search_metaheuristic = (
		routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)  # method - GUIDED_LOCAL_SEARCH

	# Searching log in console
	search_parameters.log_search = False

	# Solve the problem
	solution = routing.SolveWithParameters(search_parameters)

	# Getting solving status
	solver_status = routing.status()
	solver_result = {}
	# Print solution on console.
	if solution:
		solver_result = {'status': solver_status, 'manager': manager, 'routing': routing, 'solution': solution}
	else:
		solver_result = {'status': solver_status}

	return solver_result


# Routing Function
def get_route(route_for_draw, day_route, day_time, data):
	# Server request
	url = server + "/route/v1/driving/" + route_for_draw + '?overview=full'

	# Trying to connect to the server
	try:
		r = requests.get(url)
		res = r.json()

		# Json response processing
		# Get geometry of route
		rt = polyline.decode(res['routes'][0]['geometry'])

		points = [{'distance': 0, 'duration': 0}]
		for a in res['routes'][0]['legs']:
			points.append({'distance': a['distance'], 'duration': a['duration'] * data['time_coefficient']})

		# Process time of departure(arrival) from(to) school
		school_time = day_time.split(':')
		time_temp = timedelta(hours=int(school_time[0]), minutes=int(school_time[1]))

		# Start time calculation
		if data['direction'] == 0:
			route_time = time_temp - timedelta(
				seconds=round(res['routes'][0]['duration'] * data['time_coefficient'] + (
							data['service_time'] * (len(points) - 2))))
		else:
			route_time = time_temp

		# Route points
		waypoints = []
		time = 0
		counter = 0
		distance_from_school = 0
		distance_to_school = res['routes'][0]['distance']
		for i in res['waypoints']:

			# Arrival hour calculation
			hour = timedelta(seconds=time) + route_time

			if counter + 1 == len(res['waypoints']):
				ind = 0
			else:
				ind = counter + 1

			if data['direction'] == 0:
				waypoints.append({'index': counter, 'id': day_route[counter]['id'], 'arrival_time': time,
								  "arrival_hour": str(hour),
								  "distance_next": round(points[ind]['distance']),
								  "time_next": round(points[ind]['duration']),
								  "school_distance": round(distance_to_school),
								  'lng': i['location'][0], 'lat': i['location'][1]})
				distance_to_school -= points[ind]['distance']
			else:
				waypoints.append({'index': counter, 'id': day_route[counter]['id'], 'arrival_time': time,
								  "arrival_hour": str(hour),
								  "distance_next": round(points[counter]['distance']),
								  "time_next": round(points[counter]['duration']),
								  "school_distance": round(distance_from_school),
								  'lng': i['location'][0], 'lat': i['location'][1]})
				distance_from_school += points[ind]['distance']

			# For last and first points no service time
			if counter == (len(res['waypoints']) - 1) or counter == 0:
				time += round(points[ind]['duration'])
			else:
				time += round(points[ind]['duration'] + data['service_time'])
			counter += 1

		start_point = {'id': day_route[0]['id'],
					   'arrival_time': 0,
					   "arrival_hour": str(route_time),
					   'lng': res['waypoints'][0]['location'][0],
					   'lat': res['waypoints'][0]['location'][1]}

		end_point = {'id': day_route[-1]['id'],
					 'arrival_time': time,
					 "arrival_hour": str(hour),
					 'lng': res['waypoints'][-1]['location'][0],
					 'lat': res['waypoints'][-1]['location'][1]}

		# Create information for drawing a map
		draw = {'distance': round(res['routes'][0]['distance']),
				'start': start_point,
				'finish': end_point,
				'waypoints': waypoints,
				#   'geometry': res['routes'][0]['geometry'],
				'polyline': rt
				}
		route_info = {'load': len(res['routes'][0]['legs']), 'total_time': time,
					  'distance': res['routes'][0]['distance'],
					  'draw': draw}
	except:
		data['error'] = 'Error de conexión! El servidor OSRM no responde, no es posible obtener la matriz de tiempo.'
		route_info = ''

	return route_info


# Function to translate data from json to solver format
def create_data_model(data_input):

	# Create a template
	data = {}
	data['time_coefficient'] = traffic_coefficient[data_input['time']]
	data['time'] = data_input['time']
	data['points_info'] = {}
	data['points_info']['all'] = []
	data['demands'] = {}
	data['demands']['all'] = [0, 0] + [1] * (len(data_input['points']))
	data['time_windows'] = {}
	data['time_windows']['all'] = []
	data['service_time'] = data_input['service_time']
	data['modeling_time'] = data_input['modeling_time']
	data['direction'] = data_input['direction']
	data['bus_types'] = data_input['bus_types']
	data['total_capacity'] = 0
	data['total_demands'] = 0
	data['school_time'] = data_input['school_time']
	data['info'] = ''
	data['error'] = 'NO'
	data['days'] = []
	data['matrix_coordinates'] = {}
	data['dict_coordinates'] = {}
	data['time_matrix'] = {}
	presence = {}
	max_time = 0

	# The maximum route time is reduced for simulation, because some routes may exceed it due to the difference in construction
	data['max_route_time'] = round(data_input['max_route_time'] * 60 * 0.9)

	# Set coordinates of school/business for query to OSRM
	coordinates = str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']) + ';'
	coordinates += str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']) + ';'

	# Create pattern for every day and add school point
	for key in data['school_time'].keys():
		data['time_windows'][key] = []
		data['points_info'][key] = []
		data['days'].append(key)
		presence.update({key: 1})
		data['dict_coordinates'][key] = coordinates
		data['time_windows'][key].append([0, data['max_route_time']])
		data['time_windows'][key].append([0, data['max_route_time']])
		data['points_info'][key].append(
			{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
			 'id': data_input['school']['id'],
			 'presence': presence})
		data['points_info'][key].append(
			{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
			 'id': data_input['school']['id'],
			 'presence': presence})

	# Set school points for set of all points
	data['points_info']['all'].append(
		{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
		 'id': data_input['school']['id'],
		 'presence': presence})
	data['points_info']['all'].append(
		{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
		 'id': data_input['school']['id'],
		 'presence': presence})

	# Set time windows for school
	data['time_windows']['all'].append([0, data['max_route_time']])
	data['time_windows']['all'].append([0, data['max_route_time']])

	# Unpack information about buses and set the "price" associated with their size
	data['vehicle_capacities'] = []
	data['vehicle_price'] = []
	for i in data_input['bus_types']:
		for a in range(i['quantity']):
			data['vehicle_price'].append(i['capacity'] * 25 + 10000)
			data['vehicle_capacities'].append(i['capacity'])
			data['total_capacity'] += i['capacity']
	data['num_vehicles'] = len(data['vehicle_capacities'])

	# Set the start and finish points
	data['starts'] = [1] * len(data['vehicle_capacities'])
	data['ends'] = [0] * len(data['vehicle_capacities'])

	# For routes from home to school
	if data['direction'] == 0:
		# Set information about points
		for a in data_input['points']:
			coord = str(a['lng']) + ',' + str(a['lat'])
			coordinates += coord + ';'

			# For route from home set time window like (max_route_time - max_route_time for point,max_route_time)
			twstart = int(data['max_route_time']) - int(a['max_route_time']) * 60

			# Set information about points for different days based on presence
			presence = {}
			for key in data['school_time'].keys():
				presence.update({key: a[key]})
				if a[key] == 1:
					data['dict_coordinates'][key] += coord + ';'
					data['time_windows'][key].append([twstart, data['max_route_time']])
					data['points_info'][key].append(
						{'coordinates_for_route': coord,
						 'id': a['id'],
						 'presence': presence})
				else:
					pass

			# For school modeling append all points
			data['time_windows']['all'].append([twstart, data['max_route_time']])
			data['points_info']['all'].append(
				{'coordinates_for_route': coord,
				 'id': a['id'],
				 'presence': presence})

	# For routes from home to school
	else:
		# Set information about points
		for a in data_input['points']:
			coord = str(a['lng']) + ',' + str(a['lat'])
			coordinates += coord + ';'

			# For route from school set time window like (0, max_route_time for point)
			twstop = int(a['max_route_time']) * 60

			# Set information about points for different days based on presence
			presence = {}
			for key in data['school_time'].keys():
				presence.update({key: a[key]})
				if a[key] == 1:
					data['dict_coordinates'][key] += coord + ';'
					data['time_windows'][key].append([0, twstop])
					data['points_info'][key].append(
						{'coordinates_for_route': coord,
						 'id': a['id'],
						 'presence': presence})
				else:
					pass

			# For school modeling append all points
			data['points_info']['all'].append(
				{'coordinates_for_route': coord,
				 'id': a['id'],
				 'presence': presence})
			data['time_windows']['all'].append([0, twstop])

	# Delete last character (;) from string to match request format
	coordinates = coordinates[:-1]
	for key in data['school_time'].keys():
		data['dict_coordinates'][key] = data['dict_coordinates'][key][:-1]

	# Time matrix request
	# For school (one matrix for all points):
	if data_input['school']['form'] == 0:
		result = time_matrix_query(data, coordinates, data_input, 'all')
		data['time_matrix']['all'] = result['time_matrix']

	# For business (several matrices for each day separately):
	elif data_input['school']['form'] == 1:
		for key in data['school_time'].keys():
			result = time_matrix_query(data, data['dict_coordinates'][key], data_input, key)
			data['time_matrix'][key] = result['time_matrix']

	# Check errors
	# If travel time from one of the points is longer than the maximum time
	if result['max_time'] >= data['max_route_time']:
		data['info'] = 'El tiempo de viaje desde el inicio hasta uno de los puntos finales es mayor que el tiempo máximo de viaje. '
		data['info'] += 'No se encontrará la solución. Verifique las coordenadas o aumente el tiempo máximo de viaje.'
		data['error'] = 'TravelTimeError'
	else:
		pass


	for key in data['school_time'].keys():
		total_demands = len(data['dict_coordinates'][key].split(';'))

		# If the capacity is less than required
		if total_demands > data['total_capacity']:
			data['info'] = 'Necesidad total es más que la capacidad total. No se encontrará la solución. Verifique el número de autobuses o auméntelo'
			data['error'] = 'CapacityError'
		else:
			pass
		# Set capacity
		data['demands'][key] = [0, 0] + [1] * len(data['dict_coordinates'][key].split(';'))
	return data


#Function to translate data from json to solver format for route with defined start and finish
def create_data_model_route(data_input):
	# Create a template
	data = {}
	data['time_coefficient'] = traffic_coefficient[data_input['time']]
	data['time'] = data_input['time']
	data['points_info'] = {}
	data['points_info']['all'] = []
	data['max_route_time'] = round(data_input['max_route_time'] * 60 * 0.9)
	data['demands'] = {}
	data['demands']['all'] = [0] + [1] * (len(data_input['points']))
	data['time_windows'] = {}
	data['time_windows']['all'] = []
	data['service_time'] = data_input['service_time']
	data['modeling_time'] = data_input['modeling_time']
	data['direction'] = data_input['direction']
	data['bus_types'] = data_input['bus_types']
	data['total_capacity'] = 0
	data['total_demands'] = 0
	data['school_time'] = data_input['school_time']
	data['info'] = ''
	data['error'] = 'NO'
	data['days'] = []
	data['matrix_coordinates'] = {}
	data['dict_coordinates'] = {}
	data['time_matrix'] = {}
	presence = {}
	max_time = 0

	# Get coordinates for query to OSRM
	coordinates =''

	for key in data['school_time'].keys():
		data['time_windows'][key] = []
		data['points_info'][key] = []
		data['days'].append(key)
		data['dict_coordinates'][key] = ''


	# Unpack information about buses and set the "price" associated with their size
	data['vehicle_capacities'] = []
	data['vehicle_price'] = []
	for i in data_input['bus_types']:
		for a in range(i['quantity']):
			data['vehicle_price'].append(i['capacity'] * 25 + 10000)
			data['vehicle_capacities'].append(i['capacity'])
			data['total_capacity'] += i['capacity']
	data['num_vehicles'] = len(data['vehicle_capacities'])

	# Set the start and finish points
	data['starts'] = [1] * len(data['vehicle_capacities'])
	data['ends'] = [0] * len(data['vehicle_capacities'])

	if data['direction'] == 0:

		index = 0
		# Set information about points
		for a in data_input['points']:

			if index == 0:
				coordinates += str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']) + ';'
				for key in data['school_time'].keys():
					data['time_windows'][key].append([0, data['max_route_time']])
					data['dict_coordinates'][key] += str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']) + ';'
					data['points_info'][key].append(
						{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
						 'id': data_input['school']['id'],
						 'presence': {key: 1}})

			coord = str(a['lng']) + ',' + str(a['lat'])
			coordinates += coord + ';'
			# For route from home set time window like (max_route_time - max_route_time for point,max_route_time)
			twstart = int(data['max_route_time']) - int(a['max_route_time']) * 60

			# set presence for different days
			presence = {}
			for key in data['school_time'].keys():
				presence.update({key: a[key]})
				if a[key] == 1:
					data['dict_coordinates'][key] += coord + ';'
					data['time_windows'][key].append([twstart, data['max_route_time']])
					data['points_info'][key].append(
						{'coordinates_for_route': coord,
						 'id': a['id'],
						 'presence': presence})
				else:
					pass
			index += 1
	else:

		index = 0

		# Set information about points
		for a in data_input['points']:

			if index == 1:
				coordinates += str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']) + ';'
				for key in data['school_time'].keys():
					data['dict_coordinates'][key] += str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']) + ';'
					data['time_windows'][key].append([0, data['max_route_time']])
					data['points_info'][key].append(
						{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
						 'id': data_input['school']['id'],
						 'presence': {key: 1}})

			coord = str(a['lng']) + ',' + str(a['lat'])
			coordinates += coord + ';'
			twstop = int(a['max_route_time']) * 60

			# Set presence for different days
			presence = {}
			for key in data['school_time'].keys():
				presence.update({key: a[key]})
				if a[key] == 1:
					data['dict_coordinates'][key] += coord + ';'
					data['time_windows'][key].append([0, twstop])
					data['points_info'][key].append(
						{'coordinates_for_route': coord,
						 'id': a['id'],
						 'presence': presence})
				else:
					pass
			index += 1

	coordinates = coordinates[:-1]  # delete last character from string to match request format
	for key in data['school_time'].keys():
		data['dict_coordinates'][key] = data['dict_coordinates'][key][:-1]

	# time matrix request

	for key in data['school_time'].keys():
		result = time_matrix_query_route(data, data['dict_coordinates'][key], data_input, key)
		data['time_matrix'][key] = result['time_matrix']

	# Check errors
	if result['max_time'] >= data['max_route_time']:
		data['info'] = 'El tiempo de viaje desde el inicio hasta uno de los puntos finales es mayor que el tiempo máximo de viaje. '
		data['info'] += 'No se encontrará la solución. Verifique las coordenadas o aumente el tiempo máximo de viaje.'
		data['error'] = 'TravelTimeError'
	else:
		pass

	for key in data['school_time'].keys():
		total_demands = len(data['dict_coordinates'][key].split(';'))
		if total_demands > data['total_capacity']:
			data['info'] = 'Necesidad total es más que la capacidad total. No se encontrará la solución. Verifique el número de autobuses o auméntelo'
			data['error'] = 'CapacityError'
		else:
			pass

		data['demands'][key] = [0] + [1] * len(data['dict_coordinates'][key].split(';'))

	return data


#Function to translate data from json to solver format for route with defined start and finish
def create_data_model_route_defined(data_input):
	# Create a template
	data = {}
	data['time_coefficient'] = traffic_coefficient[data_input['time']]
	data['time'] = data_input['time']
	data['points_info'] = {}
	data['points_info']['all'] = []
	data['max_route_time'] = round(data_input['max_route_time'] * 60 * 0.9)
	data['demands'] = {}
	data['time_windows'] = {}
	data['service_time'] = data_input['service_time']
	data['modeling_time'] = data_input['modeling_time']
	data['direction'] = data_input['direction']
	data['bus_types'] = data_input['bus_types']
	data['total_capacity'] = 0
	data['total_demands'] = 0
	data['school_time'] = data_input['school_time']
	data['info'] = ''
	data['error'] = 'NO'
	data['days'] = []
	data['matrix_coordinates'] = {}
	data['dict_coordinates'] = {}
	data['time_matrix'] = {}
	presence = {}
	max_time = 0

	# Get coordinates for query to OSRM
	if data['direction'] == 1:
		coordinates = str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']) + ';'
	else:
		coordinates = ''

	for key in data['school_time'].keys():
		data['time_windows'][key] = []
		data['points_info'][key] = []
		data['days'].append(key)
		data['dict_coordinates'][key] = coordinates
		if data['direction'] == 1:
			data['time_windows'][key].append([0, data['max_route_time']])
			data['points_info'][key].append(
			{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
			 'id': data_input['school']['id'],
			 'presence': {key: 1}})
		else:
			pass

	# Unpack information about buses and set the "price" associated with their size
	data['vehicle_capacities'] = []
	data['vehicle_price'] = []
	for i in data_input['bus_types']:
		for a in range(i['quantity']):
			data['vehicle_price'].append(i['capacity'] * 25 + 10000)
			data['vehicle_capacities'].append(i['capacity'])
			data['total_capacity'] += i['capacity']
	data['num_vehicles'] = len(data['vehicle_capacities'])

	# Set the start and finish points
	data['starts'] = [1] * len(data['vehicle_capacities'])
	data['ends'] = [0] * len(data['vehicle_capacities'])

	if data['direction'] == 0:
		# Set information about points
		for a in data_input['points']:

			coord = str(a['lng']) + ',' + str(a['lat'])
			coordinates += coord + ';'
			# For route from home set time window like (max_route_time - max_route_time for point,max_route_time)
			twstart = int(data['max_route_time']) - int(a['max_route_time']) * 60

			# set presence for different days
			for key in data['school_time'].keys():
				data['dict_coordinates'][key] += coord + ';'
				data['time_windows'][key].append([twstart, data['max_route_time']])
				data['points_info'][key].append(
						{'coordinates_for_route': coord,
						 'id': a['id']})

	else:
		# Set information about points
		for a in data_input['points']:
			coord = str(a['lng']) + ',' + str(a['lat'])
			coordinates += coord + ';'
			twstop = int(a['max_route_time']) * 60

			# Set presence for different days
			for key in data['school_time'].keys():
				data['dict_coordinates'][key] += coord + ';'
				data['time_windows'][key].append([0, twstop])
				data['points_info'][key].append(
						{'coordinates_for_route': coord,
						 'id': a['id']})

	for key in data['school_time'].keys():
		if data['direction'] == 1:
			pass
		else:
			data['time_windows'][key].append([0, data['max_route_time']])
			data['points_info'][key].append(
				{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
				 'id': data_input['school']['id'],
				 'presence': {key: 1}})

	coordinates = coordinates[:-1]  # delete last character from string to match request format

	for key in data['school_time'].keys():
		data['dict_coordinates'][key] = data['dict_coordinates'][key][:-1]

	for key in data['school_time'].keys():
		total_demands = len(data['dict_coordinates'][key].split(';'))
		if total_demands > data['total_capacity']:
			data['info'] = 'Necesidad total es más que la capacidad total. No se encontrará la solución. Verifique el número de autobuses o auméntelo'
			data['error'] = 'CapacityError'
		else:
			pass

	return data


# Function to create time matrix for schools and empresas
def time_matrix_query(data, coordinates, data_input, key):
	# Time matrix query
	url_req = server + '/table/v1/driving/' + coordinates + '?annotations=duration'
	result = {}
	max_time = 0
	try:
		get_matrix = requests.post(url_req)
		r = get_matrix.json()

		# Time matrix process
		for a in range(len(data['points_info'][key])):

			# Direction to school (start at arbitrary point)
			if data['direction'] == 0:
				for b in range(len(data['points_info'][key])):
					if r['durations'][a][b] == 0:
						pass
					else:
						# Edit time matrix
						if a <= 1 or b <= 1:
							r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1)
						else:
							r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1) + data[
								'service_time']
					# Checking maximum route time from the start to end points of the route
					if r['durations'][0][a] > max_time:
						max_time = r['durations'][a][b]
					else:
						pass

				# Arbitrary start point
				r['durations'][1][a] = 0
				r['durations'][a][1] = 0

			# Direction from school (finish at arbitrary point)
			else:
				for b in range(len(data['points_info'][key])):
					if r['durations'][a][b] == 0:
						pass
					else:

						# Edit time matrix
						if a <= 1 or b <= 1:
							r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1)
						else:
							r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1) + data[
								'service_time']

					# Checking the maximum distance of the start and end points of the route
					if r['durations'][0][a] > max_time:
						max_time = r['durations'][a][b]
					else:
						pass

				# Arbitrary finish point
				r['durations'][0][a] = 0
				r['durations'][a][0] = 0

		# Write changing matrix to
		result['time_matrix'] = r['durations']

		result['max_time'] = max_time

	# If could not connect to OSRM server
	except:
		data['error'] = 'Error de conexión! El servidor OSRM no responde, no es posible obtener la matriz de tiempo.'
	return result


# Function to create time matrix for route with defined start and finish
def time_matrix_query_route(data, coordinates, data_input, key):
	# Time matrix query
	url_req = server + '/table/v1/driving/' + coordinates + '?annotations=duration'
	result = {}
	max_time = 0
	try:
		get_matrix = requests.post(url_req)
		r = get_matrix.json()


		# Time matrix process
		for a in range(len(data['points_info'][key])):
			for b in range(len(data['points_info'][key])):
				if r['durations'][a][b] == 0:
					pass
				else:
					# Edit time matrix
					if a < 1 or b < 1:
						r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1)
					else:
						r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1) + data[
							'service_time']
				# Checking maximum route time from the start to end points of the route
				if r['durations'][0][a] > max_time:
					max_time = r['durations'][a][b]
				else:
					pass

		# Write changing matrix to
		result['time_matrix'] = r['durations']

		result['max_time'] = max_time

	# If could not connect to OSRM server
	except:
		data['error'] = 'Error de conexión! El servidor OSRM no responde, no es posible obtener la matriz de tiempo.'
	return result


# Output the result in the required format for schools
def print_solution(data, solver_result, data_input):
	manager = solver_result['manager']
	routing = solver_result['routing']
	solution = solver_result['solution']
	serv_result = {}
	base_results = {}
	serv_result['status'] = ''
	serv_result['info'] = {}
	serv_result['days'] = []
	base_results['routes'] = []
	time_dimension = routing.GetDimensionOrDie('Time')
	total_bus = 0
	total_capacity = 0
	route_id = 0
	used_buses = ''
	
	# Create log depending of direction
	if data['direction'] == 0:
		serv_result['info']['direction'] = 'Recogida'
	else:
		serv_result['info']['direction'] = 'Entrega'
	
	# Print solution for every bus
	for vehicle_id in range(data['num_vehicles']):
		index = routing.Start(vehicle_id)
		route_distance = 0
		route_load = 0
		point_index = 1
		temp_route = []
		route = []
				
		# Different first point depending on the direction of route
		if data['direction'] == 0:
			pass
		else:
			route.append(data['points_info']['all'][1])
		
		# Write route 
		while not routing.IsEnd(index):
			time_var = time_dimension.CumulVar(index)
			node_index = manager.IndexToNode(index)
			route_load += data['demands']['all'][node_index]
			previous_index = index
			index = solution.Value(routing.NextVar(index))
			route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

			if route_distance != 0:
				temp_route.append(solution.Min(time_var))
			else:
				pass
			# Skipping the first section, because the route is not circular
			if node_index == 1:
				pass
			else:
				route.append(data['points_info']['all'][node_index])
			point_index += 1
		
		# Different last point depending on the direction
		if data['direction'] == 0:
			route.append(data['points_info']['all'][1])
		else:
			pass
			
		# If the route exists, then save to base results
		if route_distance != 0:
			base_results['routes'].append({
				'id': route_id,
				'route': route,
				'bus_capacity': data['vehicle_capacities'][vehicle_id],
				'load': route_load,
				'total_time': solution.Min(time_var)})

			used_buses += str(data['vehicle_capacities'][vehicle_id])+','
			total_bus += 1
			total_capacity += data['vehicle_capacities'][vehicle_id]
			route_id += 1
		else:
			pass

	# Add bus information
	used_buses = used_buses[:-1]
	used_buses = used_buses.split(',')
	serv_result['info']['used_buses'] = {}
	for i in range(len(data['bus_types'])):
		bus = str(data['bus_types'][i]['capacity'])
		serv_result['info']['used_buses'].update({bus: used_buses.count(bus)})

	# Transpose base result to other days
	for key in data['school_time'].keys():
		day_routes = []
		total_demands = 0
		total_load = 0
		total_time = 0
		total_distance = 0
		total_points = 0

		day_time = data['school_time'][key]

		for a in base_results['routes']:
			day_index = 0
			route_for_draw = ''
			day_route = []
			for i in a['route']:
				if i['presence'][key] == 1:
					route_for_draw += i['coordinates_for_route'] + ';'
					day_route.append(i)
				else:
					pass
			if (len(day_route) == 1):
				total_bus -= 1

			else:
				route_for_draw = route_for_draw[:-1]
				info = {'id': a['id'], 'bus_capacity': a['bus_capacity']}
				route = get_route(route_for_draw, day_route, day_time, data)
				info.update(route)

				total_demands += info['load']
				total_load += info['load']
				total_points += info['load']
				total_time += info['total_time']
				total_distance += info['distance']

				day_routes.append(info)
				day_index += 1

		if day_index > 0:
			serv_result['days'].append({key:
				{'summary': {
					"number_of_routes": total_bus,
					"total_demands": total_demands,
					"total_load": total_load,
					"total_capacity": total_capacity,
					"total_time": round(total_time),
					"total_distance": round(total_distance),
					"total_points": total_points,
					"school_time": day_time},
					"routes": day_routes}})
		else:
			pass

	return serv_result


# Output the result in the required format for empresas
def print_solution_empresa(data, solver_result, data_input, key):
	manager = solver_result['manager']
	routing = solver_result['routing']
	solution = solver_result['solution']
	serv_result = {}
	base_results = {}
	serv_result['status'] = ''
	serv_result['info'] = {}
	serv_result['days'] = []
	base_results['routes'] = []
	time_dimension = routing.GetDimensionOrDie('Time')
	total_bus = 0
	total_capacity = 0
	route_id = 0
	used_buses = ''

	# Print solution for every bus
	for vehicle_id in range(data['num_vehicles']):
		index = routing.Start(vehicle_id)
		route_distance = 0
		route_load = 0
		point_index = 1
		temp_route = []
		route = []

		# Different first point depending on the direction of route
		if data['direction'] == 0:
			pass
		else:
			route.append(data['points_info'][key][1])

		# Write route
		while not routing.IsEnd(index):
			time_var = time_dimension.CumulVar(index)
			node_index = manager.IndexToNode(index)
			route_load += data['demands'][key][node_index]
			previous_index = index
			index = solution.Value(routing.NextVar(index))
			route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

			if route_distance != 0:
				temp_route.append(solution.Min(time_var))
			else:
				pass
			# Skipping the first section, because the route is not circular
			if node_index == 1:
				pass
			else:
				route.append(data['points_info'][key][node_index])
			point_index += 1

		# Different last point depending on the direction
		if data['direction'] == 0:
			route.append(data['points_info'][key][1])
		else:
			pass

		# If the route exists, then save to base results
		if route_distance != 0:
			base_results['routes'].append({
				'id': route_id,
				'route': route,
				'bus_capacity': data['vehicle_capacities'][vehicle_id],
				'load': route_load,
				'total_time': solution.Min(time_var)})

			used_buses += str(data['vehicle_capacities'][vehicle_id]) + ','
			total_bus += 1
			total_capacity += data['vehicle_capacities'][vehicle_id]
			route_id += 1

		else:
			pass

	# Add bus information
	used_buses = used_buses[:-1]
	used_buses = used_buses.split(',')
	buses = {}
	for i in range(len(data['bus_types'])):
		bus = str(data['bus_types'][i]['capacity'])
		buses.update({bus: used_buses.count(bus)})

	day_routes = []
	total_demands = 0
	total_load = 0
	total_time = 0
	total_distance = 0
	total_points = 0
	day_time = data['school_time'][key]

	# Process base relults to final format
	for a in base_results['routes']:
		day_index = 0
		route_for_draw = ''
		day_route = []
		for i in a['route']:
			if i['presence'][key] == 1:
				route_for_draw += i['coordinates_for_route'] + ';'
				day_route.append(i)
			else:
				pass
		if (len(day_route) == 1):
			total_bus -= 1

		else:
			route_for_draw = route_for_draw[:-1]
			info = {'id': a['id'], 'bus_capacity': a['bus_capacity']}
			route = get_route(route_for_draw, day_route, day_time, data)
			info.update(route)

			total_demands += info['load']
			total_load += info['load']
			total_points += info['load']
			total_time += info['total_time']
			total_distance += info['distance']

			day_routes.append(info)
			day_index += 1


	serv_result = {'summary':{
				"used_buses": buses,
				"number_of_routes": total_bus,
				"total_demands": total_demands,
				"total_load": total_load,
				"total_capacity": total_capacity,
				"total_time": round(total_time),
				"total_distance": round(total_distance),
				"total_points": total_points,
				"school_time": day_time},
				"routes": day_routes}


	return serv_result


# Output the result in the required format for route with defined start and finish
def print_solution_empresa_route(data, solver_result, data_input, key):
	manager = solver_result['manager']
	routing = solver_result['routing']
	solution = solver_result['solution']
	serv_result = {}
	base_results = {}
	serv_result['status'] = ''
	serv_result['info'] = {}
	serv_result['days'] = []
	base_results['routes'] = []
	time_dimension = routing.GetDimensionOrDie('Time')
	total_bus = 0
	total_capacity = 0
	route_id = 0
	used_buses = ''

	# Print solution for every bus
	for vehicle_id in range(data['num_vehicles']):
		index = routing.Start(vehicle_id)
		route_distance = 0
		route_load = 0
		temp_route = []
		route = []

		# Write route
		while not routing.IsEnd(index):
			time_var = time_dimension.CumulVar(index)
			node_index = manager.IndexToNode(index)
			route_load += data['demands'][key][node_index]
			previous_index = index
			index = solution.Value(routing.NextVar(index))
			route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

			if route_distance != 0:
				temp_route.append(solution.Min(time_var))
			else:
				pass

			route.append(data['points_info'][key][node_index])

		route.append(data['points_info'][key][0])

		# If the route exists, then get its parameters from the server for drawing on the map
		if route_distance != 0:
			base_results['routes'].append({
				'id': route_id,
				'route': route,
				'bus_capacity': data['vehicle_capacities'][vehicle_id],
				'load': route_load,
				'total_time': solution.Min(time_var)})

			used_buses += str(data['vehicle_capacities'][vehicle_id]) + ','
			total_bus += 1
			total_capacity += data['vehicle_capacities'][vehicle_id]
			route_id += 1
		else:
			pass

	# Add bus information
	used_buses = used_buses[:-1]
	used_buses = used_buses.split(',')
	buses = {}
	for i in range(len(data['bus_types'])):
		bus = str(data['bus_types'][i]['capacity'])
		buses.update({bus: used_buses.count(bus)})

	day_routes = []
	total_demands = 0
	total_load = 0
	total_time = 0
	total_distance = 0
	total_points = 0
	day_time = data['school_time'][key]

	# Process base relults to final format
	for a in base_results['routes']:
		day_index = 0
		route_for_draw = ''
		day_route = []
		for i in a['route']:
			if i['presence'][key] == 1:
				route_for_draw += i['coordinates_for_route'] + ';'
				day_route.append(i)
			else:
				pass
		if (len(day_route) == 1):
			total_bus -= 1

		else:
			route_for_draw = route_for_draw[:-1]
			info = {'id': a['id'], 'bus_capacity': a['bus_capacity']}
			route = get_route(route_for_draw, day_route, day_time, data)
			info.update(route)

			total_demands += info['load']
			total_load += info['load']
			total_points += info['load']
			total_time += info['total_time']
			total_distance += info['distance']

			day_routes.append(info)
			day_index += 1


	serv_result = {'summary':{
				"used_buses": buses,
				"number_of_routes": total_bus,
				"total_demands": total_demands,
				"total_load": total_load,
				"total_capacity": total_capacity,
				"total_time": round(total_time),
				"total_distance": round(total_distance),
				"total_points": total_points,
				"school_time": day_time},
				"routes": day_routes}


	return serv_result


# Output the result in the required format for route with defined start and finish
def print_solution_empresa_route_defined(data, key):
	serv_result = {}
	serv_result['status'] = ''
	serv_result['info'] = {}
	serv_result['days'] = []
	route = []

	for a in data['points_info'][key]:
		route.append(a)

	day_routes = []
	total_demands = 0
	total_load = 0
	total_time = 0
	total_distance = 0
	total_points = 0
	day_time = data['school_time'][key]

	route_for_draw = ''
	day_route = []
	for i in route:
		route_for_draw += i['coordinates_for_route'] + ';'
		day_route.append(i)

	route_for_draw = route_for_draw[:-1]
	info = {'id': 0, 'bus_capacity': 0}
	route = get_route(route_for_draw, day_route, day_time, data)
	info.update(route)

	total_demands += info['load']
	total_load += info['load']
	total_points += info['load']
	total_time += info['total_time']
	total_distance += info['distance']

	day_routes.append(info)
	ind = 0
	error_pass = 'Excedido el tiempo máximo de viaje para siguientes pasajeros:'
	error_pass_log = ''
	warning = []

	if round(total_time) > data['max_route_time']/0.9:
		warning.append({'code': 'MaxTimeError', 'message':'Excedido el tiempo máximo de ruta'})

	#print(info['draw']['waypoints'][0])

	if data['direction'] == 0:
		for a in info['draw']['waypoints']:
			passanger_time = info['total_time'] - a['arrival_time']
			max_time = data['time_windows'][key][ind][1]-data['time_windows'][key][ind][0]
			if passanger_time > max_time and data['points_info'][key][ind]['id']['point_id'] != 0 and data['points_info'][key][ind]['id']['person_id'] != 0:
				error_pass_log += str(data['points_info'][key][ind]['id'])
			else:
				pass
			ind += 1
	else:
		for a in info['draw']['waypoints']:
			passanger_time = a['arrival_time']
			max_time = data['time_windows'][key][ind][1]
			if passanger_time > max_time and data['points_info'][key][ind]['id']['point_id'] != 0 and data['points_info'][key][ind]['id']['person_id'] != 0:
				error_pass_log += str(data['points_info'][key][ind]['id'])
			else:
				pass
			ind += 1

	if len(error_pass_log)>0:
		warning.append({'code': 'MaxTimeErrorPassenger', 'message': 'Excedido el tiempo máximo de viaje para siguientes pasajeros:' + error_pass_log})

	serv_result = {
		'warning': warning,
		'result': {
			'summary': {
				"used_buses": 1,
				"number_of_routes": 1,
				"total_demands": total_demands,
				"total_load": total_load,
				"total_capacity": 0,
				"total_time": round(total_time),
				"total_distance": round(total_distance),
				"total_points": total_points,
				"school_time": day_time},
			"routes": day_routes}}

	return serv_result


# Calling functions for query work
app = Application([MakeModeling], 'spyne.examples.django', in_protocol=JsonDocument(), out_protocol=Soap11(),)

service = csrf_exempt(DjangoApplication(app))

application = WsgiApplication(app)
