from django.views.decorators.csrf import csrf_exempt

from spyne.server.django import DjangoApplication
from spyne.model.primitive import Unicode, Integer, AnyDict
from spyne.service import ServiceBase
from spyne.protocol.json import JsonDocument

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

#Query module
class MakeModeling(ServiceBase):
	@rpc(AnyDict, AnyDict, Integer, Integer, Integer, AnyDict, Integer, Integer, AnyDict, _returns=AnyDict)
	def vrp_modeling(ctx, school, bus_types, max_route_time, direction, time, school_time, service_time, modeling_time, points):
		solution = {}
		gotomodeling = 0
		er = 0
		
		#Check the correctness of data for modeling
		try:
			data_input = {'school':school,
					'bus_types': bus_types,
					'max_route_time': max_route_time,
					'direction': direction,
					'time': time,
					'school_time': school_time,
					'service_time':service_time,
					'modeling_time':modeling_time,
					'points':points}

			data = create_data_model(data_input)

				#If the data matches the template, then go to the modeling
			if data['error'] == 'NO':
				gotomodeling = 1

				#If the data is correct, but modeling is not possible for them, then we raise other errors
			elif data['error'] == 'TravelTimeError':
				solution['status'] = {'code': data['error'], 'message': data['info']}
				er = 1
			elif data['error'] == 'CapacityError':
				solution['status'] = {'code': data['error'], 'message': data['info']}
				er = 1
			else:
				solution = {'status': {'code': 'ConnectionError', 'message': 'OSRM server is not responding. It is not possible to get the matrix.'}}
		
		#If the data is incorrect, then we raise various types of errors
		except TypeError:
			solution = {'status': {'code': 'TypeError', 'message': 'InvalidQuery. One of the data element in query does not exist or does not match the pattern. Verify the request is correct.'}}
		except LookupError:
			solution = {'status': {'code': 'LookupError', 'message': 'InvalidQuery. One of the keys in query does not exist or does not match the pattern. Verify the request is correct.'}}
		except:
			#If the error is due to the impossibility of modeling, then do not raise the unknown error related to the data
			if er == 1:
				pass
			else:
				solution = {'status': {'code': 'UnknownDataERROR', 'message': 'Unknown error with input data'}}
		
		#Call the solution function
		if gotomodeling == 1:
			solver_result = main(data)
			
			#If a solution was found, then output
			if solver_result['status'] == 1:
				solution = print_solution(data, solver_result)
				solution['status'] = {'code': 'OK', 'message': 'Problem solved successfully'}
			
			#If not, then we raise various errors related to the solver
			elif solver_result['status'] == 2: 
				solution = {'status': {'code': 'NoSolutionERROR', 'message':'No solution found to the problem. Verify the data is correct or try to increase capacity'}}
			elif solver_result['status'] == 3:	
				solution = {'status': {'code': 'TimeERROR', 'message': 'No solution found to the problem. Time limit reached before finding a solution. Verify the data is correct or try to increase capacity'}}
			elif solver_result['status'] == 4:	
				solution = {'status': {'code': 'ModelERROR', 'message': 'No solution found to the problem. Model is not correct. Verify the data is correct'}}
			else:
				solution = {'status': {'code': 'UnknownSolverERROR', 'message': 'Unknown error'}}
		else:
			pass
		return solution

#Function to translate data from json to solver format
def create_data_model(data_input):

	#Create a template
	data = {}
	data['time_coefficient'] = traffic_coefficient[data_input['time']]
	data['time'] = data_input['time']
	data['points_info'] = []
	data['max_route_time'] = data_input['max_route_time']*60
	data['demands'] = [0, 0]+[1] * (len(data_input['points']))
	data['time_windows'] = []
	data['service_time'] = data_input['service_time']
	data['modeling_time'] = data_input['modeling_time']
	data['direction'] = data_input['direction']
	data['bus_types'] = data_input['bus_types']
	data['total_capacity'] = 0
	data['total_demands'] = len(data_input['points'])
	data['school_time'] = data_input['school_time']
	data['info'] = ''
	data['error'] = 'NO'

	max_time = 0

	# Unpack information about buses and set the "price" associated with their size
	data['vehicle_capacities'] = []
	data['vehicle_price'] = []
	for i in data_input['bus_types']:
		for a in range(i['quantity']):
			data['vehicle_price'].append(i['capacity']*25+2500)
			data['vehicle_capacities'].append(i['capacity'])
			data['total_capacity'] += i['capacity']
	data['num_vehicles'] = len(data['vehicle_capacities'])

	# Set the start and finish points
	data['starts'] = [1] * len(data['vehicle_capacities'])
	data['ends'] = [0] * len(data['vehicle_capacities'])
	
	# Get coordinates for query to OSRM
	coordinates = str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']) + ';'
	coordinates += str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']) + ';'

	# Set school points for solver
	data['points_info'].append(
		{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
		'id': data_input['school']['id'],
		'presence': {'lun': 1, 'mar': 1, 'mie': 1, 'jue': 1, 'vie': 1}})
	data['points_info'].append(
		{'coordinates_for_route': str(data_input['school']['lng']) + ',' + str(data_input['school']['lat']),
		 'id': data_input['school']['id'],
		 'presence': {'lun': 1, 'mar': 1, 'mie': 1, 'jue': 1, 'vie': 1}})

	data['time_windows'].append([0, data['max_route_time']])
	data['time_windows'].append([0, data['max_route_time']])

	if data['direction'] == 0:
		# Set information about points
		for a in data_input['points']:
			coord = str(a['lng']) + ',' + str(a['lat'])
			coordinates += coord + ';'
			# For route from home set time window like (max_route_time - max_route_time for point,max_route_time)
			twstart = int(data['max_route_time']) - int(a['max_route_time'])*60
			data['points_info'].append(
				{'coordinates_for_route': coord,
				 'id': a['id'],
				 'presence': {'lun': a['lun'], 'mar': a['mar'], 'mie': a['mie'], 'jue': a['jue'], 'vie': a['vie']}})
			data['time_windows'].append([twstart, data['max_route_time']])

	else:
		for a in data_input['points']:
			# Set information about points
			coord = str(a['lng']) + ',' + str(a['lat'])
			coordinates += coord + ';'
			# For route from school set time window like (0, max_route_time for point)
			twstop = int(a['max_route_time'])*60
			data['points_info'].append(
				{'coordinates_for_route': coord,
				 'id': a['id'],
				 'presence': {'lun': a['lun'], 'mar': a['mar'], 'mie': a['mie'], 'jue': a['jue'], 'vie': a['vie']}})
			data['time_windows'].append([0, twstop])

	coordinates = coordinates[:-1]  # delete last character from string to match request format

	# Time matrix query
	url_req = server + '/table/v1/driving/' + coordinates + '?annotations=duration'
	try:
		get_matrix = requests.post(url_req)
		r = get_matrix.json()	

	# Time matrix process
		for a in range(len(data_input['points'])):

			# Direction to school (start at arbitrary point)
			if data['direction'] == 0:
				for b in range(len(data_input['points'])+2):			
					if r['durations'][a][b] == 0:
						pass
					else:
						# Edit time matrix
						if a <= 1 or b <= 1:
							r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1)
						else:
							r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1) + data['service_time']
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
				for b in range(len(data_input['points'])+2):
					if r['durations'][a][b] == 0:
						pass
					else:
						
						# Edit time matrix
						if a <= 1 or b <= 1:
							r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1)
						else:
							r['durations'][a][b] = round(r['durations'][a][b] * data['time_coefficient'], 1) + data['service_time']
					
					# Checking the maximum distance of the start and end points of the route
					if r['durations'][0][a] > max_time: 
						max_time = r['durations'][a][b]
					else:
						pass
					
				# Arbitrary finish point
				r['durations'][0][a] = 0
				r['durations'][a][0] = 0

		# Write changing matrix to
		data['time_matrix'] = r['durations']

		# Check errors
		if max_time >= data['max_route_time']:
			data['info'] = 'The travel time from the start to one of the end points is more than maximum travel time. '
			data['info'] += 'The solution will not be found. Check the coordinates or increase the maximum travel time.'
			data['error'] = 'TravelTimeError'
		else:
			pass
		if data['total_demands'] > data['total_capacity']:
			data['info'] = 'Total demand is more than total capacity. '
			data['info'] += 'The solution will not be found. Check the number of buses or increase it'
			data['error'] = 'CapacityError'
		else:
			pass
	# If could not connect to OSRM server
	except:
		data['error'] = 'Connection error! OSRM server is not responding, it is not possible to get the time matrix.'
	return data

# Solver
def main(data):
	
	# Create the routing index manager.
	manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['starts'], data['ends'])

	# Create Routing Model.
	routing = pywrapcp.RoutingModel(manager)

	# Create and register a transit callback.
	def distance_callback(from_index, to_index):
		# Returns the distance between the two nodes
		# Convert from routing variable Index to distance matrix NodeIndex.
		from_node = manager.IndexToNode(from_index)
		to_node = manager.IndexToNode(to_index)
		return data['time_matrix'][from_node][to_node]

	transit_callback_index = routing.RegisterTransitCallback(distance_callback)

	# Define cost of each arc.
	routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

	# Add Capacity constraint.
	def demand_callback(from_index):
		"""Returns the demand of the node."""
		# Convert from routing variable Index to demands NodeIndex.
		from_node = manager.IndexToNode(from_index)
		return data['demands'][from_node]

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
	for location_idx, time_window in enumerate(data['time_windows']):
		if location_idx == 0:
			continue
		index = manager.NodeToIndex(location_idx)
		time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
	# Add time window constraints for each vehicle start node.
	for vehicle_id in range(data['num_vehicles']):
		index = routing.Start(vehicle_id)
		time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0], data['time_windows'][0][1])

	# Instantiate route start and end times to produce feasible times.
	for i in range(data['num_vehicles']):
		routing.AddVariableMinimizedByFinalizer(
			time_dimension.CumulVar(routing.Start(i)))
		routing.AddVariableMinimizedByFinalizer(
			time_dimension.CumulVar(routing.End(i)))
		routing.SetFixedCostOfVehicle(data['vehicle_price'][i], i) # Create penalty for using extra bus

	# Setting first solution heuristic.
	search_parameters = pywrapcp.DefaultRoutingSearchParameters()
	search_parameters.time_limit.seconds = data['modeling_time']
	search_parameters.first_solution_strategy = (
		routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC) # method - PATH_CHEAPEST_ARC

	# Adding local search method for escape from local extremal points
	search_parameters.local_search_metaheuristic = (
		routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH) # method - GUIDED_LOCAL_SEARCH
	
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
	url = server + "/route/v1/driving/" + route_for_draw # + '?overview=full'

	# Trying to connect to the server
	try:
		r = requests.get(url)
		res = r.json()

		# Json response processing
		# Get geometry of route
		rt = polyline.decode(res['routes'][0]['geometry'])

		points = [{'distance': 0, 'duration': 0}]
		for a in res['routes'][0]['legs']:
			points.append({'distance': a['distance'], 'duration': a['duration']})

		# Process time of departure(arrival) from(to) school
		school_time = day_time.split(':')
		time_temp = timedelta(hours=int(school_time[0]), minutes=int(school_time[1]))

		# Start time calculation
		if data['direction'] == 0:
			route_time = time_temp - timedelta(seconds=round(res['routes'][0]['duration'] + (data['service_time'] * (len(res['waypoints'])-1))))
		else:
			route_time = time_temp

		# Route points
		waypoints = []
		time = 0
		counter = 0
		for i in res['waypoints']:
			# For last point no service time
			if counter == (len(res['waypoints'])-1):
				time += round(points[counter]['duration'])
			else:
				time += round(points[counter]['duration'] + data['service_time'])
			# Arrival hour calculation
			hour = timedelta(seconds=time) + route_time

			if counter == 0:
				waypoints.append({'index': 0, 'id': day_route[counter]['id'], 'arrival_time': 0,
								"arrival_hour": str(hour),
								'lng': i['location'][0], 'lat': i['location'][1]})
			else:
				waypoints.append({'index': counter, 'id': day_route[counter]['id'], 'arrival_time': time,
								"arrival_hour": str(hour),
								'lng': i['location'][0], 'lat': i['location'][1]})
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
		draw = {'distance': res['routes'][0]['distance'],
			   'start': start_point,
			   'finish': end_point,
			   'waypoints': waypoints,
			#   'geometry': res['routes'][0]['geometry'],
			   'polyline': rt
			   }
		route_info = {'load': len(res['routes'][0]['legs']), 'total_time': time, 'distance': res['routes'][0]['distance'],'draw': draw}
	except:
		data['error'] = 'Connection error! OSRM server is not responding, it is not possible to get the route.'
		route_info = ''

	return route_info


# Output the result in the required format
def print_solution(data, solver_result):
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
	
	#Print solution for every bus
	for vehicle_id in range(data['num_vehicles']):
		index = routing.Start(vehicle_id)
		route_distance = 0
		route_load = 0
		point_index = 1
		temp_route = []
		route = []
				
		#Different first point depending on the direction of route
		if data['direction'] == 0:
			pass
		else:
			route.append(data['points_info'][1])
		
		# Write route 
		while not routing.IsEnd(index):
			time_var = time_dimension.CumulVar(index)
			node_index = manager.IndexToNode(index)
			route_load += data['demands'][node_index]
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
				route.append(data['points_info'][node_index])
			point_index += 1
		
		# Different last point depending on the direction
		if data['direction'] == 0:
			route.append(data['points_info'][1])
		else:
			pass
			
		# If the route exists, then get its parameters from the server for drawing on the map
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

	# Add the full route information to the database
	used_buses = used_buses[:-1]
	used_buses = used_buses.split(',')
	serv_result['info']['used_buses'] = {}
	for i in range(len(data['bus_types'])):
		bus = str(data['bus_types'][i]['capacity'])
		serv_result['info']['used_buses'].update({bus: used_buses.count(bus)})

	day_keys = {1: 'lun', 2: 'mar', 3: 'mie', 4: 'jue', 5: 'vie'}
	for days in range(1, 6):
		day_routes = []
		total_demands = 0
		total_load = 0
		total_time = 0
		total_distance = 0
		total_points = 0
		day = day_keys[days]
		day_time = data['school_time'][day]

		for a in base_results['routes']:
			route_for_draw = ''
			day_route = []
			for i in a['route']:
				if i['presence'][day] == 1:
					route_for_draw += i['coordinates_for_route'] + ';'
					day_route.append(i)
				else:
					pass
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

		serv_result['days'].append({day:
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

	
	return serv_result

# Calling functions for query work
app = Application([MakeModeling], 'spyne.examples.django', in_protocol= JsonDocument(), out_protocol= JsonDocument())

service = csrf_exempt(DjangoApplication(app))
