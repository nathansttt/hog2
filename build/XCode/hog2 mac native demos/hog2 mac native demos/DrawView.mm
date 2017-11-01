//
//  DrawView.m
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 7/30/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#import "DrawView.h"

@implementation DrawView


-(id)initWithCoder:(NSCoder *)coder
{
	self = [super initWithCoder:coder];
	if (self) {
		[self getHeights];
	}
	return self;
}

- (id)initWithFrame:(NSRect)frame
{
	self = [super initWithFrame:frame];
	if (self) {
		[self getHeights];
	}
	return self;
}


-(void)getHeights
{
	height = self.frame.size.height;
	width = self.frame.size.width;
	
	double limit = std::min(self.bounds.size.width, self.bounds.size.height);
	xoffset = limit/2+(width-limit)/2;
	yoffset = limit/2+(height-limit)/2;
	xscale = limit/2.0;
	yscale = limit/2.0;
}

- (void)drawRect:(NSRect)rect
{
	[self getHeights];

	CGContextRef context = [NSGraphicsContext currentContext].CGContext;
	if (_display == 0)
	{
		CGContextSetRGBFillColor(context, 1, 0, 0, 1.0);
		CGContextFillRect(context, CGRectMake(0, 0, width, height));
		return;
	}

	CGContextSetRGBFillColor(context, 0, 0, 0, 1.0);
	CGContextFillRect(context, CGRectMake(0, 0, width, height));
	
	for (int x = 0; x < _display->drawCommands.size(); x++)
	{
		switch (_display->drawCommands[x].what)
		{
			case Graphics::Display::kLine:
			{
				Graphics::Display::lineInfo &o = _display->drawCommands[x].line;
				CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
				CGContextSetLineWidth(context, o.width);
				CGContextSetLineCap(context, kCGLineCapRound);
				CGContextMoveToPoint(context, ((o.start.x)*xscale+xoffset), height-(o.start.y*yscale+yoffset));
				CGContextAddLineToPoint(context, ((o.end.x)*xscale+xoffset), height-(o.end.y*yscale+yoffset));
				if (o.arrow)
				{
					Graphics::point newEnd = o.end*0.975f+o.start*0.025f;
					Graphics::point p1 = o.end-o.start;
					Graphics::point p2 = o.start;
					p2.z = 1;
					p2 = p1*p2;
					p2.normalise();
					p2 *= (o.end-newEnd).length();
					CGContextMoveToPoint(context, ((o.end.x)*xscale+xoffset), height-(o.end.y*yscale+yoffset));
					CGContextAddLineToPoint(context, ((newEnd.x+p2.x)*xscale+xoffset), height-((newEnd.y+p2.y)*yscale+yoffset));
					CGContextMoveToPoint(context, ((o.end.x)*xscale+xoffset), height-(o.end.y*yscale+yoffset));
					CGContextAddLineToPoint(context, ((newEnd.x-p2.x)*xscale+xoffset), height-((newEnd.y-p2.y)*yscale+yoffset));
				}
				CGContextStrokePath(context);
				break;
			}
			case Graphics::Display::kFillRectangle:
			{
				Graphics::Display::drawInfo &o = _display->drawCommands[x].shape;
				CGContextSetRGBFillColor(context, o.c.r, o.c.g, o.c.b, 1.0);
				CGContextSetLineWidth(context, o.width);
				Graphics::rect &tmp = o.r;
				CGContextFillRect(context, [self makeRect:tmp]);
				break;
			}
			case Graphics::Display::kFrameRectangle:
			{
				Graphics::Display::drawInfo &o = _display->drawCommands[x].shape;
				CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
				CGContextSetLineWidth(context, o.width);
				CGContextSetLineCap(context, kCGLineCapRound);
				Graphics::rect &tmp = o.r;
				CGContextStrokeRect(context, [self makeRect:tmp]);
				break;
			}
			case Graphics::Display::kFillOval:
			{
				Graphics::Display::drawInfo &o = _display->drawCommands[x].shape;
				CGContextSetRGBFillColor(context, o.c.r, o.c.g, o.c.b, 1.0);
				CGContextSetLineWidth(context, o.width);
				Graphics::rect &tmp = o.r;
				CGContextFillEllipseInRect(context, [self makeRect:tmp]);
				break;
			}
			case Graphics::Display::kFrameOval:
			{
				Graphics::Display::drawInfo &o = _display->drawCommands[x].shape;
				
				CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
				CGContextSetLineWidth(context, o.width);
				CGContextSetLineCap(context, kCGLineCapRound);
				Graphics::rect &tmp = o.r;
				CGContextStrokeEllipseInRect(context, [self makeRect:tmp]);
				break;
			}
		}
	}
	
}
const float epsilon = 0.5f; // in screen pixels
- (CGRect)makeRect:(Graphics::rect)r
{
	CGRect result = CGRectMake((r.left*xscale+xoffset), height-(r.top*yscale+yoffset), ((r.right-r.left)*xscale), -((r.bottom-r.top)*yscale));
	result = CGRectInset(result, -epsilon, -epsilon);
	return result;
}

-(point3d)convertToHogCoordinate:(NSPoint)currPoint
{
	point3d p;
	p.x = (currPoint.x-xoffset)/xscale;
	p.y = (height-currPoint.y-yoffset)/yscale;
//height-currPoint.y = (p.y*yscale+yoffset+epsilon)
	return p;
	//point3d p = [drawingView convertToHogCoordinate:curPoint];
}
-(BOOL)acceptsFirstResponder
{
	return YES;
}

- (void)keyDown:(NSEvent *)event
{
	[super keyDown:event];
	//	NSString *characters;
//	characters = [event characters];
//	printf("%c : %d\n", [characters characterAtIndex:0], [characters characterAtIndex:0]);
	
}


@end
