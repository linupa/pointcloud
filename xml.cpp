#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "xml.h"

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

XmlNode::XmlNode(void)
{
	sprintf(name, "---");
	rot		= VectorXd::Zero(4);
	pos		= VectorXd::Zero(3);
	com		= VectorXd::Zero(3);
	mass	= 0;
	childs	= NULL;
}
void XmlNode::parseXML(const TiXmlNode *myXML)
{
	if ( !myXML ) return;

	XmlNode *Child;
	TiXmlNode* pChildXML;
	TiXmlText* pTextXML;
	int num;
	bool skip = false;
		
	do 
	{
		int t = myXML->Type();
		bool hasChild = (myXML->FirstChild() != NULL);

		switch ( t )
		{
		case TiXmlNode::TINYXML_DOCUMENT:
			cerr << "Document " << hasChild << endl;
			if ( hasChild )
				parseXML(myXML->FirstChild());
			break;
		case TiXmlNode::TINYXML_DECLARATION:
			cerr << "declaration " << hasChild << endl;
			if ( hasChild )
				parseXML(myXML->FirstChild());
			break;
		case TiXmlNode::TINYXML_ELEMENT:
			if ( strcmp(myXML->Value(),"dynworld") == 0 )
			{
				cerr << "dynworld " << hasChild << endl;
				parseXML(myXML->FirstChild());
			}
			else
			if ( strcmp(myXML->Value(),"baseNode") == 0 )
			{
				cerr << "baseNode" << endl;
				parseXML(myXML->FirstChild());
			}
			else if ( strcmp(myXML->Value(),"jointNode") == 0 )
			{
				// Create New Joint Node
				cerr << "Create New Node" << endl;
				XmlNode *child = new XmlNode;
				child->parseXML(myXML->FirstChild());
//				childs.push_back(child);
				childs = child;
			}
			else if ( strcmp(myXML->Value(), "jointName") == 0 )
			{
				const char *pName = (myXML->FirstChild())->Value();
				cerr << "Joint Name: " << pName << endl;
				strncpy(name, pName, 20);
				
			}
			else if ( strcmp(myXML->Value(),"pos") == 0 )
			{
				int i = 0;
				char name[50], *t;
				strcpy(name, myXML->FirstChild()->Value());
				VectorXd v(3);

				t = strtok(name, "t ");
				while ( t != NULL )
				{
					v(i++) = atof(t);
					t = strtok(NULL, "t ");
				}

				cerr << "Position " << endl << v << endl;
				pos = v;
			}
			else if ( strcmp(myXML->Value(),"rot") == 0 )
			{
				int i = 0;
				char name[50], *t;
				strcpy(name, myXML->FirstChild()->Value());
				VectorXd v(4);

				t = strtok(name, "t ");
				while ( t != NULL )
				{
					v(i++) = atof(t);
					t = strtok(NULL, "t ");
				}

				cerr << "Rotation " << endl << v << endl;
				rot = v;
			}
			else if ( strcmp(myXML->Value(),"com") == 0 )
			{
				int i = 0;
				char name[50], *t;
				strcpy(name, myXML->FirstChild()->Value());
				VectorXd v(3);

				t = strtok(name, "t ");
				while ( t != NULL )
				{
					v(i++) = atof(t);
					t = strtok(NULL, "t ");
				}

				cerr << "CoM " << endl << v << endl;
				com = v;
			}
			else if ( strcmp(myXML->Value(),"mass") == 0 )
			{
				int i = 0;
				char name[50], *t;
				strcpy(name, myXML->FirstChild()->Value());

				mass= atof(name);

				cerr << "Mass " << endl << mass << endl;
			}
			break;
		}
	}
	while ( myXML = myXML->NextSibling() );

}

using namespace std;

const unsigned int NUM_INDENTS_PER_SPACE=2;

const char * getIndent( unsigned int numIndents )
{
	static const char * pINDENT="                                      + ";
	static const unsigned int LENGTH=strlen( pINDENT );
	unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
	if ( n > LENGTH ) n = LENGTH;

	return &pINDENT[ LENGTH-n ];
}

// same as getIndent but no "+" at the end
const char * getIndentAlt( unsigned int numIndents )
{
	static const char * pINDENT="                                        ";
	static const unsigned int LENGTH=strlen( pINDENT );
	unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
	if ( n > LENGTH ) n = LENGTH;

	return &pINDENT[ LENGTH-n ];
}

void dump_to_stdout( TiXmlNode* pParent, unsigned int indent)
{
	if ( !pParent ) return;

	TiXmlNode* pChild;
	TiXmlText* pText;
	int t = pParent->Type();
	int num;
	bool skip = false;
		
	switch ( t )
	{
	case TiXmlNode::TINYXML_DOCUMENT:
		printf( "%s", getIndent(indent));
		printf( "Document" );
		printf( "\n" );
		break;
			
	case TiXmlNode::TINYXML_ELEMENT:
		if ( ( strcmp(pParent->Value(),"rot") == 0 ) ||
			( strcmp(pParent->Value(),"pos") == 0 ) ||
			( strcmp(pParent->Value(),"baseNode") == 0 ) ||
			( strcmp(pParent->Value(),"dynworld") == 0 ) ||
			 ( strcmp(pParent->Value(),"jointNode") == 0 ) )
		{
			printf( "%s", getIndent(indent));
			printf( "Element [%s]", pParent->Value() );
			printf( "\n" );
		}
		else
		{
			skip = true;
		}
#if 0
		num=dump_attribs_to_stdout(pParent->ToElement(), indent+1);
		switch(num)
		{
			case 0:  printf( " (No attributes)"); break;
			case 1:  printf( "%s1 attribute", getIndentAlt(indent)); break;
			default: printf( "%s%d attributes", getIndentAlt(indent), num); break;
		}
#endif
		break;
				
#if 0
	case TiXmlNode::TINYXML_COMMENT:
		printf( "%s", getIndent(indent));
		printf( "Comment: [%s]", pParent->Value());
		printf( "\n" );
		break;
#endif
		
	case TiXmlNode::TINYXML_UNKNOWN:
		printf( "%s", getIndent(indent));
		printf( "Unknown" );
		printf( "\n" );
		break;
			
	case TiXmlNode::TINYXML_TEXT:
		pText = pParent->ToText();
		printf( "%s", getIndent(indent));
		printf( "Text: [%s]", pText->Value() );
		printf( "\n" );
		break;
				
	case TiXmlNode::TINYXML_DECLARATION:
		printf( "%s", getIndent(indent));
		printf( "Declaration" );
		printf( "\n" );
		break;
	default:
		break;
	}
	if ( !skip )
	{
		for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
		{
			dump_to_stdout( pChild, indent+1 );
		}
	}
}
				
	
