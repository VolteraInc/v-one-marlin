

// Define missing function: placement-new
inline void * operator new (size_t, void * ptr) { return ptr; }
