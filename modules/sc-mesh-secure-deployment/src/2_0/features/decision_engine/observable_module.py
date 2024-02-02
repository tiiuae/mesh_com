class ObservableModule:
    """
    A base class for modules that exhibit observable behavior, allowing them to notify an observer about certain events.

    This class is designed to be inherited by modules that require the ability to notify a decision engine
    or any other observer about specific data or occurrences.
    """
    def __init__(self, observer=None):
        """
        Initializes an ObservableModule with an optional observer.

        Parameters:
        observer: An optional observer object that will be notified of events.
                  The observer should implement an 'update' method.
        """
        self.observer = observer

    def notify(self, data):
        """
        Notifies the observer with the given data.

        This method checks if the observer exists and has an 'update' method,
        and if so, it calls the 'update' method of the observer with the provided data.

        Parameters:
        data: The data to be sent to the observer. This could be any data structure
              or information that the observer is expecting to receive.
        """
        if self.observer and hasattr(self.observer, 'update'):
            self.observer.update(data)