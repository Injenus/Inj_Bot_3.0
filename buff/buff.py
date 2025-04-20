
    def loop(self):
        
        with self.fruit_lock:
            curr_fruit_data = self.fruit_classif
            if len(curr_fruit_data) > 0:
                if curr_fruit_data['x'] >= 0.48:
                    self.main_state = 3


        if self.main_state == 0:
            with self.init_lock:
                if self.init_init == 1:
                    self.main_state = 1


        elif self.main_state == 2:
            self.send_border_mode(1)

        elif self.main_state == 3:
            self.send_border_mode(0)