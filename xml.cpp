#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "xml.h"
#include "kin.h"

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
	child	= NULL;
	sibling	= NULL;
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
				XmlNode *_ch = new XmlNode;
				_ch->parseXML(myXML->FirstChild());

				if ( child == NULL )
				{
					child = _ch;
				}
				else
				{
					XmlNode *_last = child;
					cerr << "Child already exists" << endl;
					while ( _last->sibling )
						_last = _last->sibling;
					_last->sibling = _ch;
				}
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

XmlLinkNode::XmlLinkNode(void)
{
	sprintf(name, "---");
	from	= VectorXd::Zero(3);
	to		= VectorXd::Zero(3);
	index	= 0;
	radius	= 0.;
	length	= 0.;
	linkList.clear();
}
void XmlLinkNode::parseXML(const TiXmlNode *myXML)
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
				linkList.clear();
				parseXML(myXML->FirstChild());
			}
			else if ( strcmp(myXML->Value(),"linkNode") == 0 )
			{
				// Create New Joint Node
				cerr << "Create New Link Node " << linkList.size() << endl;
				XmlLinkNode *link = new XmlLinkNode;
				link->parseXML(myXML->FirstChild());
				linkList.push_back(link);
			}
			else if ( strcmp(myXML->Value(), "linkName") == 0 )
			{
				const char *pName = (myXML->FirstChild())->Value();
				cerr << "Link Name: " << pName << endl;
				strncpy(name, pName, 20);
				
			}
			else if ( strcmp(myXML->Value(),"from") == 0 )
			{
				int i = 0;
				char name[50], *t;
				strcpy(name, myXML->FirstChild()->Value());
				VectorXd v(3);

				t = strtok(name, ", ");
				while ( t != NULL )
				{
					v(i++) = atof(t);
					t = strtok(NULL, ", ");
				}

				cerr << "From " << endl << v << endl;
				from = v;
			}
			else if ( strcmp(myXML->Value(),"to") == 0 )
			{
				int i = 0;
				char name[50], *t;
				strcpy(name, myXML->FirstChild()->Value());
				VectorXd v(3);

				t = strtok(name, ", ");
				while ( t != NULL )
				{
					v(i++) = atof(t);
					t = strtok(NULL, ", ");
				}

				cerr << "To " << endl << v << endl;
				to = v;
			}
			else if ( strcmp(myXML->Value(),"neighbor") == 0 )
			{
				int i = 0;
				char name[50], *t;
				strcpy(name, myXML->FirstChild()->Value());
				vector<int>  vv;

				vv.clear();
				t = strtok(name, ", ");
				while ( t != NULL )
				{
					vv.push_back( atoi(t) );
					t = strtok(NULL, ", ");
				}

				neighbor = VectorXi::Zero(vv.size());
				for ( i = 0 ; i < vv.size() ; i++ )
					neighbor(i) = vv[i];
				cerr << "Neighbor " << endl << neighbor.transpose() << endl;
			}
			else if ( strcmp(myXML->Value(),"index") == 0 )
			{
				int i = 0;
				char name[50], *t;
				strcpy(name, myXML->FirstChild()->Value());

				index= atoi(name);

				cerr << "Index " << endl << index << endl;
			}
			else if ( strcmp(myXML->Value(),"radius") == 0 )
			{
				int i = 0;
				char name[50], *t;
				strcpy(name, myXML->FirstChild()->Value());

				radius= atof(name);

				cerr << "Radius " << endl << radius << endl;
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
				
	
