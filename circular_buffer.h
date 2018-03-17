#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

template<typename T, uint16_t _size>
class Circular_Buffer {
    public:

        void push_back(T value) { return write(value); }
        void push_front(T value);
        T pop_front() { return read(); }
        T pop_back();
        void write(T value);
        void push_back(const T *buffer, uint16_t length) { write(buffer, length); }
        void write(const T *buffer, uint16_t length);
        void push_front(const T *buffer, uint16_t length);
        T peek(uint16_t pos = 0);
        T peekBytes(T *buffer, uint16_t length);
        T read();
        T pop_front(T *buffer, uint16_t length) { readBytes(buffer,length); }
        T read(T *buffer, uint16_t length) { readBytes(buffer,length); }
        T readBytes(T *buffer, uint16_t length);
        void flush();
        void print(const char *p);
        void println(const char *p);
        uint16_t size() { return available(); }
        uint16_t available();

    protected:
    private:
        uint16_t head = 0, tail = 0, _available = 0;
        T _cbuf[_size];
};

template<typename T, uint16_t _size>
void Circular_Buffer<T,_size>::print(const char *p) {
  for ( uint16_t i = 0; i < strlen(p); i++ ) write(p[i]);
}
template<typename T, uint16_t _size>
void Circular_Buffer<T,_size>::println(const char *p) {
  for ( uint16_t i = 0; i < strlen(p); i++ ) write(p[i]);
  write('\n');
}

template<typename T, uint16_t _size>
void Circular_Buffer<T,_size>::push_front(const T *buffer, uint16_t length) {
  for ( uint16_t i = length-1; i > 0; i-- ) push_front(buffer[i]);
  push_front(buffer[0]);
}

template<typename T, uint16_t _size>
T Circular_Buffer<T,_size>::pop_back() {
  _available--;
  tail = (tail - 1)&(2*_size-1);
  return _cbuf[tail&(_size-1)];
}

template<typename T, uint16_t _size>
void Circular_Buffer<T,_size>::push_front(T value) {
  head = (head - 1)&(2*_size-1);
  _cbuf[head&(_size-1)] = value;
  if ( _available++ >= _size ) _available = _size;
}

template<typename T, uint16_t _size>
uint16_t Circular_Buffer<T,_size>::available() {
  return _available;
}

template<typename T, uint16_t _size>
void Circular_Buffer<T,_size>::flush() {
  return head = tail = _available = 0;
}

template<typename T, uint16_t _size>
void Circular_Buffer<T,_size>::write(T value) {
  if ( _available++ >= _size ) _available = _size;
  _cbuf[tail&(_size-1)] = value;
  if ( tail == (head ^ _size) ) head = (head + 1)&(2*_size-1);
  tail = (tail + 1)&(2*_size-1);
}

template<typename T, uint16_t _size>
void Circular_Buffer<T,_size>::write(const T *buffer, uint16_t length) {
  if ( ( _available += length ) >= _size ) _available = _size;
  if ( length < ( _size - tail ) ) {
    memmove(_cbuf+tail,buffer,length*sizeof(T));
    tail = (tail + length)&(2*_size-1);
  }
  else for ( uint16_t i = 0; i < length; i++ ) write(buffer[i]);
}

template<typename T, uint16_t _size>
T Circular_Buffer<T,_size>::read() {
  T value = _cbuf[head&(_size-1)];
  head = (head + 1)&(2*_size-1);
  _available--;
  return value;
}

template<typename T, uint16_t _size>
T Circular_Buffer<T,_size>::peek(uint16_t pos) {
  if ( pos > _size ) return 0;
  return _cbuf[(head+pos)&(_size-1)];
}

template<typename T, uint16_t _size>
T Circular_Buffer<T,_size>::peekBytes(T *buffer, uint16_t length) {
  uint16_t _count;
  ( available() < length ) ? _count = available() : _count = length;
  if ( _count < ( _size - head ) ) memmove(buffer,_cbuf,_count*sizeof(T));
  else for ( uint16_t i = 0; i < _count; i++ ) buffer[i] = peek(i);
  return _count;
}

template<typename T, uint16_t _size>
T Circular_Buffer<T,_size>::readBytes(T *buffer, uint16_t length) {
  _available -= length;
  uint16_t _count;
  ( available() < length ) ? _count = available() : _count = length;
  if ( _count < ( _size - head ) ) {
    memmove(buffer,_cbuf,_count*sizeof(T));
    head = (head + _count)&(2*_size-1);
  }
  else for ( uint16_t i = 0; i < _count; i++ ) buffer[i] = read();
  return _count;
}






#endif // Circular_Buffer_H
