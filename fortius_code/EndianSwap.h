#ifndef ENDIANSWAP_H
#define ENDIANSWAP_H


// swapping function specialized for integer types
template <typename T> void swapI ( T& a, T& b )
{
	a ^= b;
	b ^= a;
	a ^= b;
}

template <size_t T> inline void endianswap_recursive(char *val)
{
	swapI<char>(*val, *(val + T - 1));
	endianswap_recursive<T - 2>(val + 1);
}

template <> inline void endianswap_recursive<0>(char *) {}
template <> inline void endianswap_recursive<1>(char *) {}

template <typename T> inline void endianswap(T& val)
{
	endianswap_recursive<sizeof(T)>((char*)(&val));
}

template<typename T> inline void ToBigEndian(T&) { }
template<typename T> inline void ToLittleEndian(T inval, T* val)
{
	*val = inval;
	//endianswap<T>(*val);
}
template<typename T> inline T FromLittleEndian(T* val)
{
	//endianswap<T>(*val);
	return *val;
}


// always convert from little to big endian and vice versa
template<typename T> inline void ToOtherEndian(T& val)
{
	endianswap<T>(val);
}

// prevent endian-converting pointers by accident
template<typename T> void EndianConvert(T*);
template<typename T> void EndianConvertReverse(T*);

// no-ops, for speed
inline void ToBigEndian(uint8_t&) { }
inline void ToBigEndian(int8_t&)  { }
inline void ToLittleEndian(uint8_t&) { }
inline void ToLittleEndian(int8_t&) { }
inline void ToOtherEndian(uint8_t&) { }
inline void ToOtherEndian(int8_t&) { }


#endif
