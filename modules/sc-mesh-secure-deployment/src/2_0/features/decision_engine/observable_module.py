class ObservableModule:
    """
    Inherited by each observable feature
    Observer = Decision Engine
    """
    def __init__(self, observer=None):
        self.observer = observer

    def notify(self, data):
        if self.observer and hasattr(self.observer, 'update'):
            self.observer.update(data)