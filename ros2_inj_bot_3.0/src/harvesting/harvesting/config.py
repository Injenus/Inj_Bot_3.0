"""
Всевозможные прааметры, ичспользущиеся в пакте
"""

dt = 0.005 # базовый период всех циклов управления

    ### arm_action  ###
dt_arm_action = 0.1 # период основго цикла обработки задач (фактически обновления очреди)
st_delay = 1.5
delay_init_safe = 0.8
delay_init = 0.2
delay_1_knock, delay_2_knock = 0.5, 0.5
delay_1_pick,delay_2_pick,delay_3_pick, delay_4_pick = 1.0, 1.2, 2.0, 2.0 
delay_throw_short = 2.5
delay_throw_long = 2.5
arm_states_table = {
        'init_safe': -2, # тупо ввкрх
        'init': -1, # вбок наблюдение
        'knock_down': 0, #сбитие
        'pick': 1, # взятие
        'throw_short_side': 2, # бросок
        '60_11': 3, # биз 60 в 11
        '11_60': 4 # биз 11 в 60
    }
                        
    ### border_move ###
lidar_r = 0.025 # радиус башки лидара
target_border_dist = 0.32 # целевое расстоние от края башки лидара до бордюра
front_turn_dist = 0.22 # растсоние спереди от края башки лиидара,Ю при котором тнаичнается поврот
base_linear_x_speed = 0.09
base_angular_w_speed = 1.2
max_abs_angular_w_speed = 3.0
min_abs_angular_w_speed = 0.0
side_p, side_i, side_d = 30, 0, 0
front_p, front_i, front_d = 20, 0, 0
tolerance_side_error = 0.007

    ### move_to_box_short_side  ###
distance_tolerance = 0.007 # допуск для всех расстояний в манёвре
wait_seconds = st_delay*(30+3) # сколько сеунд ждать после инита броска
front_dist_to_get_middle_cell = 1.2 # расстоние до переднего бордюра, чтобы оказаться в нужном положении в средней ячейки на парвой стороне !!!УЧТИ ЛИДАР И ГАБАРИТЫ!!!
back_dist_to_get_middle_cell = 1.6 # расстояние до заднего барьера, чтобы оказаться в нужном положении в средней ячейки на левой стороне !!!УЧТИ ЛИДАР И ГАБАРИТЫ!!!
back_dist_to_turn_to_box = 0.7 # целевое от ящика через зад для поврота
back_dist_to_box = 0.35 # целевое до ящика через зад зщадним ходом
left_lidar_dist_to_turn_to_box = 0.25 # крутимся пока не растсоние от левого борртт ждо коробки
front_dist_to_go_away_from_box = 0.6 # вращемчс покав не дсотгнеем этого растсония
front_dist_to_go_to_border = 0.28 # едем пока не это растсоние

    ### start_finish    ###
target_front_dist_start = 2.3   #2.3
target_back_dist_finish = 2.8

    ### cv  ###
all_classes = ['sweet_pepper_good','sweet_pepper_bad',
               'lemon_good','lemon_bad',
               'tomate_good','tomate_bad',
               'eggplant_good','eggplant_bad',
               'pear_good','pear_bad',
               ]
id_to_class = {0: 'eggplant_good',
               1: 'tomate_good',
               2: 'pear_good',
               3: 'sweet_pepper_good',
               4: 'lemon_good',
               5: 'lemon_bad',
               6: '',
               7: '',
               8: '',
               9: '',
}
matching = {
  'sweet_pepper_good': 'knock_down',
  'lemon_good': 'knock_down',

  'pear_good': 'ignore',
  'tomate_good': 'ignore',
  'eggplant_good': 'ignore',

  'lemon_bad': 'pick',
}