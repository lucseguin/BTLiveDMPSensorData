
template <typename T> class EMAFilter {
public:
    EMAFilter(float alpha) {
        _alpha = alpha;
        _filt = 0;
    }
    T filter(T value) {
        _filt = (1.0 - _alpha)*_filt + _alpha*value;
        return _filt;
    }
 
private:
    EMAFilter() {
    };
    
    float _alpha;
    T _filt;
};

