/**
 * XML Serialization
 * Simple and lightweight xml serialization class
 *
 * Original code by Lothar Perr
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any
 * damages arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any
 * purpose, including commercial applications, and to alter it and
 * redistribute it freely
 */

#pragma once
#include <string>
#include <vector>
#include <map>
#include "tinyxml2.h"

using namespace std;

#define RootClassName "SerializableClass"
#define XMLClassEndTag string("</")+RootClassName+">"
#define Empty_String string("")

/**
    XML Serialization
*/
namespace xmls
{
    class Serializable;

/**
    serializable members base class
*/
    class MemberBase
    {
    protected:
        string m_sValue;
    public:
        virtual ~MemberBase() {};
        string toString(){return m_sValue;};
        const char* c_str() {return m_sValue.c_str();};
        string *getStringPtr() {return &m_sValue;};
    };

/**
    serializable string
*/
    class xString: public MemberBase
    {
    private:
        void AssignValue(const string value){m_sValue=value;};
    public:
        xString(){};
        xString(string value) {AssignValue(value);};
        string value(){return m_sValue;};
        xString operator=(const string value) {AssignValue(value);return *this;};
        xString operator=(const char* value) {AssignValue(value);return *this;};
    };

/**
    serializable int
*/
    class xInt: public MemberBase
    {
    private:
        void AssignValue(const int value);
    public:
        xInt() {AssignValue(0);};
        xInt(int value) {AssignValue(value);};
        int value();
        xInt operator=(const int value) {AssignValue(value);return *this;};
    };

    /**
        serializable float
    */
        class xFloat: public MemberBase
        {
        private:
            void AssignValue(const float value);
        public:
            xFloat() {AssignValue(0);};
            xFloat(float value) {AssignValue(value);};
            float value();
            xFloat operator=(const float value) {AssignValue(value);return *this;};
        };

/**
    serializable bool
*/
    class xBool: public MemberBase
    {
    private:
        void AssignValue(const bool value){m_sValue = value ? "true":"false";};
    public:
        xBool() {AssignValue(false);};
        xBool(bool value) {AssignValue(value);};
        bool value();
        xBool operator=(const bool value) {AssignValue(value);return *this;};
    };

/**
    serializable time_t
*/
    class xTime_t: public MemberBase
    {
    private:
        void AssignValue(const time_t value);
    public:
        xTime_t() {AssignValue(0);};
        xTime_t(time_t value) {AssignValue(value);};
        time_t value();
        xTime_t operator=(const time_t value) {AssignValue(value);return *this;};
    };

/**
    Member-Field mapping
*/
    class FieldMapping;
    typedef vector<FieldMapping*>::iterator FieldMappingIterator;
    class FieldMapping
    {
    public:
        string m_sFieldName;
        string *m_sField;
        FieldMapping(string FieldName, string *Field) {m_sFieldName=FieldName; m_sField=Field;};
        string getFieldName(){return m_sFieldName;};
        string *getField(){return m_sField;};
    };

/**
    Member-Class mapping
*/
    class ClassMapping;
    typedef vector<ClassMapping*>::iterator ClassMappingIterator;
    class ClassMapping
    {
    public:
        string m_sMemberName;
        Serializable *m_cSubclass;
        ClassMapping(string MemberName, Serializable *subclass) {m_sMemberName=MemberName; m_cSubclass=subclass;};
        string getName(){return m_sMemberName;};
        Serializable *getSubclass(){return m_cSubclass;};
    };

/**
    Class-collection base
*/
    class CollectionBase;
    typedef vector<CollectionBase*>::iterator CollectionIterator;
    class CollectionBase
    {
        friend class Serializable;
    private:
        string m_sCollectionName;
        string m_sCollectionClassType;
    protected:
        CollectionBase() {};
        vector<Serializable*> m_vCollection;
        map<Serializable*, bool> m_mOwner;
        void setCollectionName(string value) {m_sCollectionName=value;};
        void setCollectionClassType(string value) {m_sCollectionClassType=value;};
        string getCollectionName(){return m_sCollectionName;};
        virtual Serializable *newElement()=0;
    public:
        virtual ~CollectionBase() {};
        size_t size() {return m_vCollection.size();};
        Serializable *getItem(int itemID) { return m_vCollection.at(itemID);};
        void Clear();
    };

/**
    Class-collection template
*/
    template <typename T>
    class Collection: public CollectionBase
    {
        friend class Serializable;
    public:
        ~Collection() {Clear();	};
        T *newElement();
        void addItem(T *item) { m_vCollection.push_back(item); m_mOwner[item]=false;};
        T *getItem(int itemID) { return static_cast<T*>(m_vCollection.at(itemID)); };
    };

/**
    create new element of type T
    @return empty object of type T
*/
    template <typename T>
    T* Collection<T>::newElement()
    {
        T* newItem = new T();
        m_vCollection.push_back(newItem);
        m_mOwner[newItem]=true;
        return newItem;
    };

/**
    Serializeable base class
    derive your serializable class from Serializable
*/
    typedef vector<Serializable*>::iterator SerializableIterator;
    class Serializable
    {
        friend class CollectionBase;
    private:
        Serializable(__attribute__((unused)) Serializable const &s) { }
        Serializable operator=(__attribute__((unused))Serializable const &s) {return *this;};
        static string strReplaceAll(string source, const string from, const string to);
    protected:
        string m_sXML;
        string m_sClassName;
        string m_sVersion;
        vector<FieldMapping*> m_FieldMappings;
        vector<ClassMapping*> m_SubclassMappings;
        vector<CollectionBase*> m_SubclassCollections;
        Serializable();
        virtual ~Serializable();
        void setClassName(string ClassName){m_sClassName=ClassName;};
        void Register(string MemberName, MemberBase *Member, string DefaultValue="");
        void Register(string MemberName, Serializable *Member);
        void Register(string CollectionName, CollectionBase *SubclassCollection);
        void Serialize(tinyxml2::XMLDocument *classDoc, tinyxml2::XMLElement *rootNode);
        static void Deserialize(Serializable *classItem, tinyxml2::XMLDocument *classDoc, tinyxml2::XMLElement *rootNode);
    public:
        static bool fromXML(string XMLSource, Serializable *classObject);
        static string IdentifyClass(string XMLSource);
        static string IdentifyClassVersion(string XMLSource);
        string getClassName(){return m_sClassName;};
        void setVersion(string value) {m_sVersion=value;};
        string getVersion() {return m_sVersion;};
        string toXML();
        bool Compare(Serializable *messageToCompare);
        static bool Clone(Serializable *source, Serializable *destination);
        bool Copy(Serializable *source);
        void Replace(string searchFor, string replaceWith, bool recursive=true);
        void Clear();
    };
}
