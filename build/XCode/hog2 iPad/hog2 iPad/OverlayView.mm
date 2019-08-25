//
//  OverlayView.m
//  Mathle
//
//  Created by Nathan Sturtevant on 8/28/09.
//  Copyright 2009 NS Software. All rights reserved.
//

#import "OverlayView.h"
#import <UIKit/UIKit.h>
#include "Common.h"

@implementation OverlayView

- (id)initWithFrame:(CGRect)frame {
    if (self = [super initWithFrame:frame]) {
        // Initialization code
		[self setOpaque:NO];
		[self setUserInteractionEnabled:false];
    }
//	x1 = -1;
	double limit = std::min(self.bounds.size.width, self.bounds.size.height);
	xoffset = limit/2;
	yoffset = limit/2;
	xscale = limit/2.0;
	yscale = limit/2.0;
	return self;
}

-(void)getHeights
{
	height = self.frame.size.height;
	width = self.frame.size.width;
	pRecContext context = getCurrentContext();
	context->windowHeight = height;
	context->windowWidth = width;
	
	double limit = std::min(self.bounds.size.width, self.bounds.size.height);
	xoffset = limit/2+(width-limit)/2;
	yoffset = limit/2+(height-limit)/2;
	xscale = limit/2.0;
	yscale = limit/2.0;
}

-(void)drawCurrentPath
{
	[self getHeights];
	CGContextRef context = UIGraphicsGetCurrentContext();//[UIGraphicsContext currentContext].CGContext;
	if (_display == 0)
	{
		CGContextSetRGBFillColor(context, 1, 0, 0, 1.0);
		CGContextFillRect(context, CGRectMake(0, 0, width, height));
		printf("Nothing to draw\n");
		return;
	}
	
//	CGContextSetRGBFillColor(context, 0, 0.5, 0, 1.0);
//	CGContextFillRect(context, CGRectMake(0, 0, width, height));
	
//	for (int x = 0; x < _display->backgroundDrawCommands.size(); x++)
//	{
//		[self drawCommand:&(_display->backgroundDrawCommands[x])];
//	}
//	for (int x = 0; x < _display->backgroundLineSegments.size(); x++)
//	{
//		[self drawSegments:&(_display->backgroundLineSegments[x])];
//	}
	
	for (int x = 0; x < _display->drawCommands.size(); x++)
	{
		[self drawCommand:&(_display->drawCommands[x])];
	}
	for (int x = 0; x < _display->lineSegments.size(); x++)
	{
		[self drawSegments:&(_display->lineSegments[x])];
	}
	for (int x = 0; x < _display->text.size(); x++)
	{
		// DRAW TEXT HERE
		NSString *s = [NSString stringWithUTF8String:_display->text[x].s.c_str()];
		//		NSLog(s);
		NSMutableParagraphStyle *style = [[NSParagraphStyle defaultParagraphStyle] mutableCopy];
		if (_display->text[x].align == Graphics::textAlignCenter)
			[style setAlignment:NSTextAlignmentCenter];
		else if (_display->text[x].align == Graphics::textAlignLeft)
			[style setAlignment:NSTextAlignmentLeft];
		UIFont *font = [UIFont fontWithName:(NSString *)@"Courier" size:(CGFloat)_display->text[x].size*height/2.0/1.1];
		NSDictionary *textAttributes =
		@{NSFontAttributeName: font,//[NSFont monospacedDigitSystemFontOfSize:_display->text[x].size*height/2.0 weight:1.0],
		  NSForegroundColorAttributeName: [UIColor colorWithRed:_display->text[x].c.r green:_display->text[x].c.g blue:_display->text[x].c.b alpha:1.0],
		  NSParagraphStyleAttributeName: style
		  };
		CGPoint p;
		p.x = [self hogToScreenX:_display->text[x].loc.x viewport:_display->text[x].viewport];
		p.y = [self hogToScreenY:_display->text[x].loc.y viewport:_display->text[x].viewport];
		//NSLog(@"%f %f", p.x, p.y);
		if (_display->text[x].align == Graphics::textAlignLeft)
			[s drawAtPoint:p withAttributes:textAttributes];
		else
			[s drawInRect:CGRectMake(p.x-width, p.y-_display->text[x].size*height/4.0, 2*width, _display->text[x].size*height/2.0) withAttributes:textAttributes];
	}
	
}
//{
//	CGContextRef context = UIGraphicsGetCurrentContext();
//	if (_display == 0)
//		return;
//
//	// Filled Rects
//	for (int x = 0; x < _display->filledRects.size(); x++)
//	{
//		Graphics::Display::drawInfo &o = _display->filledRects[x];
//		CGContextSetRGBFillColor(context, o.c.r, o.c.g, o.c.b, 1.0);
//		CGContextSetLineWidth(context, o.width);
//		Graphics::rect &tmp = o.r;
//		CGContextFillRect(context, [self makeRect:tmp]);
//	}
//
//	// Framed Rects
//	for (int x = 0; x < _display->framedRects.size(); x++)
//	{
//		Graphics::Display::drawInfo &o = _display->framedRects[x];
//		CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
//		CGContextSetLineWidth(context, o.width);
//		CGContextSetLineCap(context, kCGLineCapRound);
//		Graphics::rect &tmp = o.r;
//		CGContextStrokeRect(context, [self makeRect:tmp]);
//	}
//
//	// Filled Circles
//	for (int x = 0; x < _display->filledCircles.size(); x++)
//	{
//		Graphics::Display::drawInfo &o = _display->filledCircles[x];
//		CGContextSetRGBFillColor(context, o.c.r, o.c.g, o.c.b, 1.0);
//		CGContextSetLineWidth(context, o.width);
//		Graphics::rect &tmp = o.r;
//		CGContextFillEllipseInRect(context, [self makeRect:tmp]);
//	}
//
//	// Framed Circles
//	for (int x = 0; x < _display->framedCircles.size(); x++)
//	{
//		Graphics::Display::drawInfo &o = _display->filledCircles[x];
//
//		CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
//		CGContextSetLineWidth(context, o.width);
//		CGContextSetLineCap(context, kCGLineCapRound);
//		Graphics::rect &tmp = o.r;
//		CGContextFillEllipseInRect(context, [self makeRect:tmp]);
//	}
//
//	// Lines
//	for (int x = 0; x < _display->lines.size(); x++)
//	{
//		Graphics::Display::lineInfo &o = _display->lines[x];
//		CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
//		CGContextSetLineWidth(context, o.width);
//		CGContextSetLineCap(context, kCGLineCapRound);
//		CGContextMoveToPoint(context, o.start.x*xscale+xoffset, o.start.y*yscale+yoffset);
//		CGContextAddLineToPoint(context, o.end.x*xscale+xoffset, o.end.y*yscale+yoffset);
//		CGContextStrokePath(context);
//	}
//
//
///*
//	// UI Line
//	if (x1 != -1)
//	{
//		printf("Drawing line from (%f, %f) to (%f, %f)\n", x1, y1, x2, y2);
//		CGContextSetRGBStrokeColor(context, 0.0, 1.0, 0.0, 0.75);
//		CGContextSetLineWidth(context, 4.0);
//		CGContextSetLineCap(context, kCGLineCapRound);
//		CGContextMoveToPoint(context, x1, y1);
//		CGContextAddLineToPoint(context, x2, y2);
//		CGContextStrokePath(context);
//	}*/
//}
/*
- (bool)setLineStartX:(float)x1s StartY:(float)y1s EndX:(float)x2g EndY:(float)y2g
{
	x1 = x1s;
	x2 = x2g;
	y1 = y1s;
	y2 = y2g;
	return true;
}

-(void)clearLine
{
	if (x1 != -1)
	{
		x1 = -1;
	}
}
*/
- (void)drawRect:(CGRect)rect {
	[self drawCurrentPath];
}
/*
- (void)addLine:(Graphics::point)start lineEnd:(Graphics::point)end lineColor:(rgbColor)c lineWeight:(float)w;
{
	drawLines.push_back({start, end, c, w});
}
- (void)clear
{
	drawLines.clear();
	drawRects.clear();
	frameRects.clear();
	drawCircle.clear();
	frameCircle.clear();
}

- (void)addRect:(Graphics::rect)r color:(rgbColor)c
{
	drawRects.push_back({r, c});
}

- (void)addFramedRect:(Graphics::rect)r color:(rgbColor)c
{
	frameRects.push_back({r, c});
}

- (void)addCircle:(Graphics::rect)r color:(rgbColor)c
{
	drawCircle.push_back({r, c});
}

- (void)addFramedCircle:(Graphics::rect)r color:(rgbColor)c
{
	frameCircle.push_back({r, c});
}
*/

-(void)drawSegments:(Graphics::Display::segments*)seg
{
	CGContextRef context = UIGraphicsGetCurrentContext();//[UIGraphicsContext currentContext].CGContext;
	Graphics::Display::segments &s = *seg;
	int port = s.viewport;
	if (s.points.size() == 0)
		return;
	CGContextSetRGBStrokeColor(context, s.c.r, s.c.g, s.c.b, 1.0);
	CGContextSetLineWidth(context, s.size);
	CGContextMoveToPoint(context, [self hogToScreenX:s.points[0].x viewport:port], [self hogToScreenY:s.points[0].y viewport:port]);
	for (int y = 1; y < s.points.size(); y++)
	{
		CGContextAddLineToPoint(context, [self hogToScreenX:s.points[y].x viewport:port], [self hogToScreenY:s.points[y].y viewport:port]);
	}
	CGContextStrokePath(context);
}

-(void)drawCommand:(Graphics::Display::data*)dat
{
	Graphics::Display::data &d = *dat;
	CGContextRef context = UIGraphicsGetCurrentContext();//[UIGraphicsContext currentContext].CGContext;
	int port = d.viewport;
	switch (d.what)
	{
		case Graphics::Display::kLine:
		{
			Graphics::Display::lineInfo &o = d.line;
			CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width);
			CGContextSetLineCap(context, kCGLineCapRound);
			printf("Line to (%f, %f)\n", [self hogToScreenX:o.start.x viewport:port],
				   [self hogToScreenY:o.start.y viewport:port]);
			CGContextMoveToPoint(context, [self hogToScreenX:o.start.x viewport:port], [self hogToScreenY:o.start.y viewport:port]);
			CGContextAddLineToPoint(context, [self hogToScreenX:o.end.x viewport:port], [self hogToScreenY:o.end.y viewport:port]);
			
			if (o.arrow)
			{
				Graphics::point newEnd = o.end*0.975f+o.start*0.025f;
				Graphics::point p1 = o.end-o.start;
				Graphics::point p2 = o.start;
				p2.z = 1;
				p2 = p1*p2;
				p2.normalise();
				p2 *= (o.end-newEnd).length();
				CGContextMoveToPoint(context, [self hogToScreenX:o.end.x viewport:port], [self hogToScreenY:o.end.y viewport:port]);
				CGContextAddLineToPoint(context, [self hogToScreenX:newEnd.x+p2.x viewport:port], [self hogToScreenY:newEnd.y+p2.y viewport:port]);
				CGContextMoveToPoint(context, [self hogToScreenX:o.end.x viewport:port], [self hogToScreenY:o.end.y viewport:port]);
				CGContextAddLineToPoint(context, [self hogToScreenX:newEnd.x-p2.x viewport:port], [self hogToScreenY:newEnd.y-p2.y viewport:port]);
			}
			CGContextStrokePath(context);
			break;
		}
		case Graphics::Display::kFillRectangle:
		{
			Graphics::Display::drawInfo &o = d.shape;
			CGContextSetRGBFillColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width);
			Graphics::rect &tmp = o.r;
			CGContextFillRect(context, [self makeRect:tmp viewport:port]);
			break;
		}
		case Graphics::Display::kFrameRectangle:
		{
			Graphics::Display::drawInfo &o = d.shape;
			CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width);
			CGContextSetLineCap(context, kCGLineCapRound);
			Graphics::rect &tmp = o.r;
			CGContextStrokeRect(context, [self makeRect:tmp viewport:port]);
			break;
		}
		case Graphics::Display::kFillNGon:
		{
			Graphics::Display::shapeInfo &o = d.polygon;
			CGContextSetRGBFillColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			double resolution = TWOPI/o.segments;
			glBegin(GL_TRIANGLE_FAN);
			for (int x = 0; x <= o.segments; x++)
			{
				CGFloat nextx, nexty;
				nextx = o.center.x+sin(resolution*x+o.rotate*TWOPI/360.0)*o.radius;
				nexty = o.center.y+cos(resolution*x+o.rotate*TWOPI/360.0)*o.radius;
				if (x == 0)
					CGContextMoveToPoint(context,
										 [self hogToScreenX:nextx viewport:port],
										 [self hogToScreenY:nexty viewport:port]);
				else
					CGContextAddLineToPoint(context,
											[self hogToScreenX:nextx viewport:port],
											[self hogToScreenY:nexty viewport:port]);
			}
			CGContextFillPath(context);
			break;
		}
		case Graphics::Display::kFillOval:
		{
			Graphics::Display::drawInfo &o = d.shape;
			CGContextSetRGBFillColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width);
			Graphics::rect &tmp = o.r;
			CGContextFillEllipseInRect(context, [self makeRect:tmp viewport:port]);
			break;
		}
		case Graphics::Display::kFrameOval:
		{
			Graphics::Display::drawInfo &o = d.shape;
			
			CGContextSetRGBStrokeColor(context, o.c.r, o.c.g, o.c.b, 1.0);
			CGContextSetLineWidth(context, o.width*50.0f);
			CGContextSetLineCap(context, kCGLineCapRound);
			Graphics::rect &tmp = o.r;
			CGContextStrokeEllipseInRect(context, [self makeRect:tmp viewport:port]);
			break;
		}
	}
	
}


const float epsilon = 0.5f; // in screen pixels
- (CGRect)makeRect:(Graphics::rect)r viewport:(int)v
{
	CGRect result = CGRectMake([self hogToScreenX:r.left viewport:v],
							   [self hogToScreenY:r.top viewport:v],
							   [self hogToScreenX:r.right viewport:v]-[self hogToScreenX:r.left viewport:v],
							   [self hogToScreenY:r.bottom viewport:v]-[self hogToScreenY:r.top viewport:v]);
	//							   (r.right-r.left)*xscale,
	//							   -((r.bottom-r.top)*yscale));
	//							   (r.left*xscale+xoffset),
	//							   height-(r.top*yscale+yoffset),
	//							   ((r.right-r.left)*xscale),
	//							   -((r.bottom-r.top)*yscale));
	result = CGRectInset(result, -epsilon, -epsilon);
	return result;
}
- (Graphics::point)getWorldCoordinate:(Graphics::point)point
{
	point.x = (point.x-xoffset)/xscale;
	point.y = (point.y-yoffset)/yscale;
	return point;
}

- (CGRect)makeRect:(Graphics::rect)r
{
	return CGRectMake(r.left*xscale+xoffset-epsilon, r.top*yscale+yoffset+epsilon, (r.right-r.left)*xscale+2*epsilon, (r.bottom-r.top)*yscale+2*epsilon);
}

-(CGFloat)hogToScreenX:(CGFloat)x viewport:(int)v
{
	pRecContext pContextInfo = getCurrentContext();
	point3d input(x, 0.f, 0.f);
	point3d result = ViewportToGlobalHOG(pContextInfo, pContextInfo->viewports[v], input);
	//	if (v == 1)
	//		printf("X:%f -> %f\n", x, ((result.x+1.0))/2.0);
	return ((result.x+1.0)*width)/2.0;
}

-(CGFloat)hogToScreenY:(CGFloat)y viewport:(int)v
{
	pRecContext pContextInfo = getCurrentContext();
	point3d input(0.f, y, 0.f);
	point3d result = ViewportToGlobalHOG(pContextInfo, pContextInfo->viewports[v], input);
	//	result.y -= pContextInfo->viewports[v].bounds.bottom;
	//	result.y = pContextInfo->viewports[v].bounds.top - result.y;
	//	if (v == 1)
	//		printf("Y:%f -> %f\n", y, ((result.y+1.0))/2.0);
	return -((result.y-1.0)*height)/2.0;
}

-(point3d)convertToGlobalHogCoordinate:(CGPoint)currPoint
{
	point3d p;
	p.x = 2.0*currPoint.x/width-1.0;
	p.y = -2.0*currPoint.y/height+1.0;
	p.z = 0;
	return p;
	
	//	switch (_display->GetNumViewports())
	//	{
	//		case 1:
	//			p.x = (currPoint.x-xoffset)/xscale;
	//			p.y = (height-currPoint.y-yoffset)/yscale;
	//			return p;
	//		case 2:
	//			int v = [self getViewport:currPoint];
	//			if (v == 0)
	//				p.y = (((height-currPoint.y)-yoffset)/yscale+0.25)/0.75;
	//			if (v == 1)
	//				p.y = (((height-currPoint.y)-yoffset)/yscale+0.75)/0.25;
	//	}
	////	(currPoint.x - xscale)/( = (0.75*p.x*xscale+xoffset);
	//	p.x = (currPoint.x-xoffset)/(xscale*0.75);
	//
	//	return p;
	//point3d p = [drawingView convertToHogCoordinate:curPoint];
}
@end
