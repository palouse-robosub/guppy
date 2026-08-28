class CodeMap:
    def __init__(self, map=None):
        self._forward = dict(map or {})
        self._reverse = {val: key for key, val in self._forward.items()}

    def __getitem__(self, key):
        return self._reverse.get(key)

    def __iter__(self):
        return iter(self._reverse)
