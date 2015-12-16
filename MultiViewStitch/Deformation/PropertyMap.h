// The PropertyMap class defines an interface for storing set of key-value pairs.
// It uses an unique associative array (std::map) to associate a value to a key.
// PropertyMap class is used to associate properties to vertex, 
// halfedge, and facet.
template<typename KeyType, typename ValueType>
class PropertyMap
{
public:
	
	typedef KeyType                      KeyType;
	typedef ValueType                    ValueType;
	typedef std::map<KeyType, ValueType> MapType;
	
public:
	
	// Set the value of the property for a given key.
	// If given key already exists then its value will be updated.
	inline void setValue(const KeyType& key, const ValueType& value)
	{
		// for std::map there are two functions to insert a value: operator[] and insert().
		// If key already exists, then insert() will not update existing value while operator[] 
		// will update the value. We want latter behavior to allow overwriting of property values.
		mPropertyMap[key] = value;
	}
	
	// Get the value associated with the given key. If there is no item with the given 
	// key, the function throws an out_of_bounds exception.
	inline const ValueType& value(const KeyType& key) const
	{
		MapType::const_iterator _iter = mPropertyMap.find(key);
		if(_iter != mPropertyMap.end())
		{
			return _iter->second;
		}
		else
		{
			throw std::out_of_range("Key doesn't exists in property map.");
		}
	}
	
	// Get the value associated with the given key. If there is no item with the given 
	// key, the function throws an out_of_bounds exception.
	inline ValueType& value(const KeyType& key)
	{
		MapType::iterator _iter = mPropertyMap.find(key);
		if(_iter != mPropertyMap.end())
		{
			return _iter->second;
		}
		else
		{
			throw std::out_of_range("Key doesn't exists in property map.");
		}
	}
	
	// Check if there is an item with a given key.
	inline bool hasKey(const KeyType& key) const
	{
		return mPropertyMap.find(key) != mPropertyMap.end();
	}
	
	// Get the size of the property map.
	inline size_t size() const
	{
		return mPropertyMap.size();
	}
	
	// Delete all items from the property map.
	inline void clear()
	{
		mPropertyMap.clear();
	}
	
public:
	
	inline typename MapType::const_iterator begin() const {return mPropertyMap.begin();}
	inline typename MapType::const_iterator end()   const {return mPropertyMap.end();}
	
private:
	
	MapType mPropertyMap; ///< Create an instance of the property map.
};
