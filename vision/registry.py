class Registry:
    def __init__(self):
        self._items = {}

    def register(self, name, factory):
        self._items[name] = factory

    def create(self, name, *args, **kwargs):
        if name not in self._items:
            raise KeyError("unknown component: %s" % name)
        return self._items[name](*args, **kwargs)
