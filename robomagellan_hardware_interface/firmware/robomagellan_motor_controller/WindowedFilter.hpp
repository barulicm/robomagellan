
template<size_t N>
class WindowedFilter
{
public:
  WindowedFilter()
  {
    for(int i = 0; i < N; ++i)
      buffer[i] = 0.0;
  }

  double filter(double raw)
  {
    tail = (tail + 1) % N;
    sum -= buffer[tail];
    sum += raw;
    buffer[tail] = raw;
    if(size < N)
      size++;
    return sum / size;
  }

private:
  double buffer[N]{};
  double sum{0.0};
  size_t tail{0};
  size_t size{0};
};
