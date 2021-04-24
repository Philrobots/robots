from handlers.handler import Handler


class EndCycleHandler(Handler):
    def handle(self, handled_data):
        handled_data["red_light_pub"].publish(True)
        return handled_data

    def unregister(self):
        pass
