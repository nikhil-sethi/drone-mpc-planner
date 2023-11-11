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

#include <algorithm>
#include <string>
#include <iostream>
#include <sstream>
#include "XMLSerialization.h"

using namespace std;

/**
XML Serialization namespace
*/
namespace xmls
{
/**
    Assign a value to the xint member
    @return	void
    */
void xInt::AssignValue(int value)
{
    stringstream ss;
    ss << value;
    m_sValue=ss.str();
};

/**
    Returns the int-value of the xint member
    @return	current value
    */
int xInt::value()
{
    stringstream ss(m_sValue);
    int value;
    if( (ss >> value).fail() )
        return 0;
    return value;
};

/**
    Assign a value to the xfloat member
    @return	void
    */
void xFloat::AssignValue(float value)
{
    stringstream ss;
    ss << value;
    m_sValue=ss.str();
};

/**
    Returns the int-value of the xint member
    @return	current value
    */
float xFloat::value()
{
    stringstream ss(m_sValue);
    float value;
    if( (ss >> value).fail() )
        return 0.f;
    return value;
};

/**
    Returns the bool-value of the xbool member
    @return	current value
    */
bool xBool::value()
{
    bool value=false;
    string sHelp =  m_sValue;
    transform(sHelp.begin(), sHelp.end(), sHelp.begin(), ::toupper);
    if (sHelp=="TRUE") return true;
    return value;
};

/**
    Assign a value to the xTime_t member
    @return	void
    */
void xTime_t::AssignValue(const time_t value)
{
    stringstream ss;
    ss << value;
    m_sValue=ss.str();
};

/**
    Delete all (self created) collection-elements
    @return void
    */
void CollectionBase::Clear()
{
    if (m_vCollection.size()>0)
    {
        for (vector<Serializable*>::iterator it = m_vCollection.begin() ; it != m_vCollection.end(); ++it)
            if (m_mOwner.find(*it)->second)
                delete(*it);
        m_vCollection.clear();
        m_mOwner.clear();
    }
}

/**
    SerializableBase Constructor
    */
Serializable::Serializable():
    m_sClassName("Serializable"),
    m_sVersion("1")
{ }

/**
    SerializableBase Destructor
    Clean Collections, Mappings and Subclass-Mappings
    */
Serializable::~Serializable()
{
    if (m_SubclassCollections.size()>0)
    {
        for (CollectionIterator it_collection = m_SubclassCollections.begin() ; it_collection != m_SubclassCollections.end(); ++it_collection)
            (*it_collection)->Clear();
        m_SubclassCollections.clear();
    }
    if (m_SubclassMappings.size()>0)
        for (ClassMappingIterator it_subClassMapping = m_SubclassMappings.begin() ; it_subClassMapping != m_SubclassMappings.end(); ++it_subClassMapping)
            delete(*it_subClassMapping);

    if (m_FieldMappings.size()>0)
        for (FieldMappingIterator it_fieldMapping = m_FieldMappings.begin() ; it_fieldMapping != m_FieldMappings.end(); ++it_fieldMapping)
            delete(*it_fieldMapping);

    m_SubclassCollections.clear();
    m_SubclassMappings.clear();
    m_FieldMappings.clear();
}

/**
    Register a member
    @MemberName XML-Description/Name for the member
    @Member Member to register
    @return void
    */
void Serializable::Register(string MemberName, MemberBase *Member, string DefaultValue)
{
    FieldMapping *mapping = new FieldMapping(MemberName, Member->getStringPtr());
    *(Member->getStringPtr())=DefaultValue;
    m_FieldMappings.push_back(mapping);
}

/**
    Register a member-subclass
    @MemberName XML-Description/Name for the member-class
    @Member Member-class to register
    @return void
    */
void Serializable::Register(string MemberName, Serializable *Member)
{
    ClassMapping *mapping = new ClassMapping(MemberName, Member);
    m_SubclassMappings.push_back(mapping);
}

/**
    Register a class-collection
    @CollectionName XML-Description/Name for the collection
    @SubclassCollection Collection to register
    @return void
    */
void Serializable::Register(string CollectionName, CollectionBase *SubclassCollection)
{
    SubclassCollection->setCollectionName(CollectionName);
    m_SubclassCollections.push_back(SubclassCollection);
}

/**
    Perform serialization
    @classDoc tinyxml Class Document
    @rootNode tinyxml Element rootNode
    @return void
    */
void Serializable::Serialize(tinyxml2::XMLDocument *classDoc, tinyxml2::XMLElement *rootNode)
{
    for (FieldMappingIterator it_member = m_FieldMappings.begin() ; it_member != m_FieldMappings.end(); ++it_member)
    {
        __attribute__((unused)) string *field = (*it_member)->getField();
        tinyxml2::XMLElement *memberNode = classDoc->NewElement("Member");
        memberNode->SetAttribute("Name", ((*it_member)->getFieldName()).c_str());
        tinyxml2::XMLText* memberValue;
        memberValue = classDoc->NewText((*(*it_member)->getField()).c_str());
        memberNode->InsertEndChild(memberValue);
        rootNode->InsertEndChild(memberNode);
    }

    for (ClassMappingIterator it_subclass = m_SubclassMappings.begin() ; it_subclass != m_SubclassMappings.end(); ++it_subclass)
    {
        Serializable *subClass = (*it_subclass)->getSubclass();
        tinyxml2::XMLElement *subClassNode = classDoc->NewElement("Class");
        subClassNode->SetAttribute("Name", ((*it_subclass)->getName()).c_str());
        subClassNode->SetAttribute("Type", (*it_subclass)->getSubclass()->getClassName().c_str());
        subClassNode->SetAttribute("Version", (*it_subclass)->getSubclass()->getVersion().c_str());
        rootNode->InsertEndChild(subClassNode);
        subClass->Serialize(classDoc, subClassNode);
    }

    for (CollectionIterator it_collection = m_SubclassCollections.begin() ; it_collection != m_SubclassCollections.end(); ++it_collection)
    {
        tinyxml2::XMLElement *listNode = classDoc->NewElement("Collection");
        listNode->SetAttribute("Name", (*it_collection)->getCollectionName().c_str());

        for (size_t c=0; c<(*it_collection)->size(); c++)
        {
            Serializable *item = (*it_collection)->getItem(c);
            tinyxml2::XMLElement *elementNode = classDoc->NewElement("Class");
            elementNode->SetAttribute("Type", item->getClassName().c_str());
            elementNode->SetAttribute("Version", item->getVersion().c_str());
            item->Serialize(classDoc, elementNode);
            listNode->InsertEndChild(elementNode);
        }
        rootNode->InsertEndChild(listNode);
    }
}

/**
    Perform deserialization
    @classItem destination object
    @classDoc tinyxml Class Document
    @rootNode tinyxml Element rootNode
    @return void
    */
void Serializable::Deserialize(Serializable *classItem, tinyxml2::XMLDocument *classDoc, tinyxml2::XMLElement *rootNode)
{
    classItem->setClassName(rootNode->Attribute("Type"));
    classItem->setVersion(rootNode->Attribute("Version"));

    tinyxml2::XMLElement *memberNode = rootNode->FirstChildElement("Member");
    while(memberNode && memberNode->GetText())
    {
        string fieldName = memberNode->Attribute("Name");
        for (FieldMappingIterator it = classItem->m_FieldMappings.begin() ; it != classItem->m_FieldMappings.end(); ++it)
            if ((*it)->getFieldName()==fieldName)
                *(*it)->getField()=memberNode->GetText();
        memberNode = memberNode->NextSiblingElement("Member");
    }

    tinyxml2::XMLElement *classNode = rootNode->FirstChildElement("Class");
    while(classNode!=NULL)
    {
        string className = classNode->Attribute("Name");

        for (ClassMappingIterator it_subclass = classItem->m_SubclassMappings.begin() ; it_subclass != classItem->m_SubclassMappings.end(); ++it_subclass)
            if ((*it_subclass)->getName()==className)
                (*it_subclass)->getSubclass()->Deserialize((*it_subclass)->getSubclass(), classDoc, classNode);
        classNode = classNode->NextSiblingElement("Class");
    }

    tinyxml2::XMLElement *collectionNode = rootNode->FirstChildElement("Collection");
    while (collectionNode!=NULL)
    {
        string collectionName = collectionNode->Attribute("Name");
        for (CollectionIterator it_collection = classItem->m_SubclassCollections.begin() ; it_collection != classItem->m_SubclassCollections.end(); ++it_collection)
        {
            if ((*it_collection)->getCollectionName()==collectionName)
            {
                tinyxml2::XMLElement *classNode_in_collection = collectionNode->FirstChildElement("Class");
                while (classNode_in_collection!=NULL)
                {
                    Serializable *newItem = (*it_collection)->newElement();
                    Serializable::Deserialize(newItem, classDoc, classNode_in_collection);
                    classNode_in_collection = classNode_in_collection->NextSiblingElement("Class");
                }
            }
        }
        collectionNode = collectionNode->NextSiblingElement("Collection");
    }
}

/**
    Perform serialization
    @return XML-Data
    */
string Serializable::toXML()
{
    tinyxml2::XMLDocument *classDoc = new tinyxml2::XMLDocument();
    tinyxml2::XMLElement *rootNode;
    rootNode = classDoc->NewElement(RootClassName);
    rootNode->SetAttribute("Type", m_sClassName.c_str());
    rootNode->SetAttribute("Version", m_sVersion.c_str());
    classDoc->InsertFirstChild(rootNode);
    Serialize(classDoc, rootNode);
    tinyxml2::XMLPrinter printer;
    classDoc->Accept(&printer);
    string xmlResult = printer.CStr();
    classDoc->Clear();
    delete(classDoc);
    return xmlResult;
}

/**
    Perform deserialization
    @XMLSource XML-Input
    @classObject (empty) destination object
    @return deserialized object
    */
bool Serializable::fromXML(string XMLSource, Serializable *classObject)
{
    bool result=false;
    tinyxml2::XMLDocument *classDoc = new tinyxml2::XMLDocument();
    tinyxml2::XMLElement *rootNode;
    if (tinyxml2::XML_SUCCESS==classDoc->Parse(XMLSource.c_str(), XMLSource.length()))
    {
        rootNode = classDoc->FirstChildElement(RootClassName);
        if (rootNode)
        {
            Deserialize(classObject, classDoc, rootNode);
            result=true;
        }
    }
    classDoc->Clear();
    delete(classDoc);
    return result;
}

/**
    Get Class-Type of XML-Source
    @XMLSource XML-Input
    @return class name
    */
string Serializable::IdentifyClass(string XMLSource)
{
    tinyxml2::XMLDocument doc;
    doc.Parse(XMLSource.c_str(), XMLSource.length());
    tinyxml2::XMLElement* rootNode;
    rootNode = doc.FirstChildElement(RootClassName);
    if (rootNode)
    {
        const char* value = rootNode->Attribute("Type");
        if (value) return string(value);
    }
    return Empty_String;
}

/**
    Get Class-Version of XML-Source
    @XMLSource XML-Input
    @return class version
    */
string Serializable::IdentifyClassVersion(string XMLSource)
{
    tinyxml2::XMLDocument doc;
    doc.Parse(XMLSource.c_str(), XMLSource.length());
    tinyxml2::XMLElement* rootNode;
    rootNode = doc.FirstChildElement(RootClassName);
    if (rootNode)
    {
        const char* value = rootNode->Attribute("Version");
        if (value) return string(value);
    }
    return Empty_String;
}

/**
    Perform deep value compare of serializable objects
    including all sub-classes and collections
    @msg compare object
    @return true the objects are equal
    */
bool Serializable::Compare(Serializable *msg)
{
    bool identical=true;
    if (getClassName()!=msg->getClassName())
        identical=false;

    if (identical)
        for (FieldMappingIterator it = m_FieldMappings.begin() ; it != m_FieldMappings.end(); ++it)
            for (FieldMappingIterator it_c = msg->m_FieldMappings.begin() ; it_c != msg->m_FieldMappings.end(); ++it_c)
                if (((*it)->getFieldName()) == ((*it_c)->getFieldName()))
                    if (*(*it)->getField()!=*(*it_c)->getField())
                        identical=false;

    if (identical)
        for (ClassMappingIterator it = m_SubclassMappings.begin() ; it != m_SubclassMappings.end(); ++it)
            for (ClassMappingIterator it_c = msg->m_SubclassMappings.begin() ; it_c != msg->m_SubclassMappings.end(); ++it_c)
                if ((*it)->getName() == (*it_c)->getName())
                    if (!(*it)->getSubclass()->Compare((*it_c)->getSubclass()))
                        identical=false;

    if (identical)
    {
        if (m_SubclassCollections.size()!=msg->m_SubclassCollections.size())
        {identical=false;}
        else
            for (CollectionIterator it = m_SubclassCollections.begin() ; it != m_SubclassCollections.end(); ++it)
                for (CollectionIterator it_c = msg->m_SubclassCollections.begin() ; it_c != msg->m_SubclassCollections.end(); ++it_c) {
                    if ((*it)->getCollectionName()==(*it_c)->getCollectionName()) {
                        if ((*it)->size()!=(*it_c)->size()) {
                            identical=false;
                        } else {
                            for (size_t c=0; c<(*it)->size(); c++) {
                                if (!(*it)->getItem(c)->Compare((*it_c)->getItem(c)))
                                { identical=false; break; }
                            }
                        }
                    }
                }
    }
    return identical;
}

/**
    Clone a serializable object
    including all sub-classes and collections
    @source source object
    @destination new/empty destination object
    @return true if the action succeeded
    */
bool Serializable::Clone(Serializable *source, Serializable *destination)
{
    bool result=true;
    if (source->getClassName()!=destination->getClassName())
        result=false;

    if (result)
        for (FieldMappingIterator it_source = source->m_FieldMappings.begin() ; it_source != source->m_FieldMappings.end(); ++it_source)
            for (FieldMappingIterator it_dest = destination->m_FieldMappings.begin() ; it_dest != destination->m_FieldMappings.end(); ++it_dest)
                if ((*it_dest)->getFieldName() == (*it_source)->getFieldName())
                    *(*it_dest)->getField() = *(*it_source)->getField();

    if (result)
        for (ClassMappingIterator it_source = source->m_SubclassMappings.begin() ; it_source != source->m_SubclassMappings.end(); ++it_source)
            for (ClassMappingIterator it_dest = destination->m_SubclassMappings.begin() ; it_dest != destination->m_SubclassMappings.end(); ++it_dest)
                if ((*it_dest)->getName() == (*it_source)->getName())
                    if (!(*it_dest)->getSubclass()->Copy((*it_source)->getSubclass()))
                        result=false;

    for (CollectionIterator it_source = source->m_SubclassCollections.begin() ; it_source != source->m_SubclassCollections.end(); ++it_source)
        for (CollectionIterator it_dest = destination->m_SubclassCollections.begin() ; it_dest != destination->m_SubclassCollections.end(); ++it_dest)
            if ((*it_source)->getCollectionName()==(*it_dest)->getCollectionName()) {
                for (size_t c=0; c<(*it_source)->size(); c++)
                {
                    Serializable *newItem = (*it_dest)->newElement();
                    if (!newItem->Copy((*it_source)->getItem(c)))
                    {result=false;}
                }
            }
    return result;
}

/**
    Copy object-data from an serializable object
    @source the source object
    @return true if the action succeeded
    */
bool Serializable::Copy(Serializable *source)
{
    return Serializable::Clone(source, this);
}

/**
    Search/Replace in a string
    @source source the source string
    @searchFor search for
    @replaceWith replace with
    @return the resulting string
    */
string Serializable::strReplaceAll(string source, const string searchFor, const string replaceWith)
{
    if(searchFor.empty())
        return source;
    size_t start_pos = 0;
    while((start_pos = source.find(searchFor, start_pos)) != string::npos) {
        source.replace(start_pos, searchFor.length(), replaceWith);
        start_pos += replaceWith.length();
    }
    return source;
}

/**
    Search/Replace all members and sub-members
    @source search for
    @replaceWith replace with
    @recursive process all sub-classes
    */
void Serializable::Replace(string searchFor, string replaceWith, bool recursive)
{
    for (FieldMappingIterator it = m_FieldMappings.begin() ; it != m_FieldMappings.end(); ++it)
        *((*it)->getField()) = strReplaceAll((*(*it)->getField()), searchFor, replaceWith);
    if (recursive)
    {
        for (ClassMappingIterator it = m_SubclassMappings.begin() ; it != m_SubclassMappings.end(); ++it)
            (*it)->getSubclass()->Replace(searchFor, replaceWith);
        for (CollectionIterator it = m_SubclassCollections.begin() ; it != m_SubclassCollections.end(); ++it)
            for (size_t c=0; c<(*it)->size(); c++) { (*it)->getItem(c)->Replace(searchFor, replaceWith); }
    }
}
}
