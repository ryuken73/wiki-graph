import React from 'react';
import {ForceGraph2D} from 'react-force-graph';
import {useWindowSize} from '@react-hook/window-size';
import {isLinkBiDirectional} from './js/graphHandlers';
import {getPersonImage} from './js/serverApi.js';
import Skeleton from '@mui/material/Skeleton';
import ActionButtons from './Components/ActionButtons.jsx';
import noImage from './assets/images/noImage.webp';
import styled from 'styled-components';

const CardContainer = styled.div`
  position: absolute;
  display: none;
  /* width: 100px; */
  z-index: 10;
  top: 0; 
  border-radius: 10px;
  /* flex-direction: column; */
  grid-template-columns: 1fr 1fr;
  color: white;
  padding-top: 5px;
  backdrop-filter: blur(10px);  
  border: 3px solid rgba(255, 255, 255, 0.2);
  padding: 10px;
  background: maroon;
  opacity: 0;
  transition: opacity 0.5s;
`
const RowContainer = styled.div`
  display: flex;
  justify-content: start;
`
const ImageContainer = styled.div`
  position: relative;
`
const Text = styled.div`
  text-align: left;
  font-weight: 100;
  font-size: 12px;
`
const ImageInfo = styled(Text)`
  color: black;
  font-size: 11px;
  font-weight: 300;
  position: absolute;
  bottom: 7px;
  /* background: rgba(0,0,0,0.5); */
  background: rgba(255,255,255,0.6);
  width: 100px;
  border-bottom-left-radius: 10px;
  border-bottom-right-radius: 10px;
  text-align: center;
`

const CustomImg = styled.img`
  /* width: 100%; */
  width: 100px;
  object-fit: cover;
  object-position: top;
  border-radius: 10px;
  aspect-ratio: 4/5;
  cursor: pointer;
`
const Contents = styled.div`
  width: fit-content;
  /* background-color: maroon; */
`

let imageHash = {};
let noImageObj;
fetch(noImage)
.then(res => res.blob())
.then(blob => {
  noImageObj = URL.createObjectURL(blob)
})

const openChildWindow = (wikiUrl, windowFeatures) => {
  console.log('open', wikiUrl)
  window.open(`https://namu.wiki${wikiUrl}`, "aa", `width=800,height=1200,${windowFeatures}`);
}

const getTextWidth = (ctx, text, font = "16px Arial") => {
    ctx.font = font;
    return ctx.measureText(text).width;
}

const getImage = async (contentId) => {
  const cached = imageHash[contentId]
  console.log(cached)
  if(cached){
    return cached;
  }
  const imgBlob = await getPersonImage(contentId)
  if(imgBlob !== null){
    const objURL = URL.createObjectURL(imgBlob)
    imageHash[contentId] = objURL;
    return objURL
  }
  imageHash[contentId] = noImageObj;
  return null;
}

function positionElement(graph, x, y, element) {
  const {x:left, y:top} = graph.graph2ScreenCoords(x, y);
  element.style.top = `${top}px`;
  element.style.left = `${left}px`;
  element.style.display = 'block';
  element.style.opacity = 1;
}

const showBox = (ctx, element, x, y) => {
  positionElement(ctx, x, y, element);
}

function Graph2D(props, graphRef) {
  const [width, height] = useWindowSize()
  const {graphData, expandNode} = props;
  const [nodeHovered, setNodeHovered] = React.useState(null);
  const [highlightNodes, setHighligntNodes] = React.useState(new Set());
  const [highlightLinks, setHighligntLinks] = React.useState(new Set());
  const [imgSrc, setImgSrc] = React.useState(noImageObj);
  const [imgLoaded, setImgLoaded] = React.useState(false);
  // const fgRef = React.useRef(null);
  const imgRef = React.useRef(null);
  const ctxRef = React.useRef(null);
  const globalScaleRef = React.useRef(9);

  const FONT_SIZE = 14;
  const FONT_WEIGHT = 300;
  const FONT_FAMILY = 'SBAggroL';

  const timerRef = React.useRef(null);
  const showCard = React.useCallback((nodeHovered) => {
    if(timerRef.current !== null){
      clearTimeout(timerRef.current);
    }
    if(nodeHovered === null){
      imgRef.current.style.display = 'none';
      setImgLoaded(false);
      setImgSrc(noImageObj);
      return;
    }
    timerRef.current = setTimeout(() => {
      getImage(nodeHovered.id)
      .then(imgObjURL => {
        if(imgObjURL){
          setImgLoaded(true);
          setImgSrc(imgObjURL);
        }
      })
      const label = nodeHovered.text;
      const fontSize = FONT_SIZE/globalScaleRef.current;
      const font = `${FONT_WEIGHT} ${fontSize}px ${FONT_FAMILY}`;
      const textWidth = getTextWidth(ctxRef.current, label, font);
      const bckgDimensions = [textWidth, fontSize].map(n => n + fontSize * 0.5); // some padding
      showBox(graphRef.current, imgRef.current, nodeHovered.x - bckgDimensions[0]/2, nodeHovered.y+bckgDimensions[1]/2)
      setNodeHovered(nodeHovered)
      timerRef.current = null;
    }, 100)
  }, [nodeHovered])
  const updateHighlight = React.useCallback(() => {
    setHighligntNodes(highlightNodes)
  }, [])

  const handleLeftClick = React.useCallback((node) => {
    const isForwarding = true;
    expandNode(node, isForwarding)
  }, [])
  const handleRightClick = React.useCallback((node) => {
    const isForwarding = false;
    expandNode(node, isForwarding)
  }, [])
  const handleLinkClick = React.useCallback((link, event) => {
    console.log('source:target-', link.source.id, link.target.id)
    const hasReverseLink = isLinkBiDirectional(link, graphData.links)
    const srcNode = link.source;
    const tgtNode = link.target;
    if(hasReverseLink){
      openChildWindow(tgtNode.wikiUrl, "right=0")
      setTimeout(() => {
        openChildWindow(srcNode.wikiUrl, "left=0")
      }, 5000)
    } else {
      openChildWindow(srcNode.wikiUrl, "right=0")
    }
  }, [])
  const handleImgClick = React.useCallback(() => {
    const tgtNode = nodeHovered 
    console.log(tgtNode)
    if(tgtNode){
      openChildWindow(tgtNode.wikiUrl, "right=0");
    }
  }, [nodeHovered])

  const handleNodeHover = React.useCallback((node) => {
    setHighligntNodes((highlightNodes) => {
      highlightNodes.clear();
      if(node){
        console.log(node, node.neighbors)
        highlightNodes.add(node);
        node.neighbors?.forEach(neighbor => highlightNodes.add(neighbor));
      }
      return highlightNodes
    })
    setHighligntLinks((highlightLinks) => {
      highlightLinks.clear();
      if(node){
        console.log(node, node.links)
        node.links?.forEach(link => highlightLinks.add(link));
      }
      return highlightLinks
    })
    showCard(node);

    // highlightNodes.clear();
    // if(node){
    //   highlightNodes.add(node);
    //   node.neighbors.forEach(neighbor => highlightNodes.add(neighbor));
    //   updateHighlight();
    // }
  }, [])

  return (
    <>
    <ForceGraph2D
      width={width}
      height={height}
      ref={graphRef}
      graphData={graphData}
      // backgroundColor="#000003"
      backgroundColor="transparent"
      linkColor={(link)=> highlightLinks.has(link)? 'rgba(234, 239, 44,0.5)': 'rgba(255,255,255,0.2)'}
      linkWidth={(link)=> highlightLinks.has(link) ? 5: 1}
      onNodeClick={handleLeftClick}
      onNodeRightClick={handleRightClick}
      onLinkClick={handleLinkClick}
      linkDirectionalArrowLength={0}
      linkDirectionalArrowRelPos={1}  
      linkDirectionalParticles={1}
      onEngineStop={() => console.log('engine stops')}
      // onEngineStop={() => graphRef.current.zoomToFit(100, 100)}
      onNodeDragEnd={node => {
        node.fx = node.x;
        node.fy = node.y;
        node.fz = node.z;
      }}
      onNodeHover={handleNodeHover}
      onRenderFramePre={(ctx, globalScale) => {
        ctxRef.current = ctx;
        globalScaleRef.current = globalScale;
      }}
      nodeCanvasObject={async (node, ctx, globalScale) => {
        const label = node.text;
        const fontSize = FONT_SIZE/globalScaleRef.current;
        ctx.font = `${FONT_WEIGHT} ${fontSize}px ${FONT_FAMILY}`;
        const textWidth = ctx.measureText(label).width;
        const bckgDimensions = [textWidth, fontSize].map(n => n + fontSize * 0.5); // some padding

        ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
        ctx.fillRect(node.x - bckgDimensions[0] / 2, node.y - bckgDimensions[1] / 2, ...bckgDimensions);

        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = node.color;
        ctx.fillText(label, node.x, node.y);

        const needHighlight = highlightNodes.has(node);
        if(needHighlight){
          ctx.lineWidth = 1 /globalScale;
          ctx.beginPath();
          ctx.strokeRect(node.x - bckgDimensions[0] / 2, node.y - bckgDimensions[1] / 2, ...bckgDimensions);
          ctx.strokeStyle = 'yellow';
        }
        node.__bckgDimensions = bckgDimensions; // to re-use in nodePointerAreaPaint
      }}
      nodePointerAreaPaint={(node, color, ctx) => {
        ctx.fillStyle = color;
        const bckgDimensions = node.__bckgDimensions;
        bckgDimensions && ctx.fillRect(node.x - bckgDimensions[0] / 2, node.y - bckgDimensions[1] / 2, ...bckgDimensions);
      }}
    >
    </ForceGraph2D>
    <CardContainer ref={imgRef}>
      <RowContainer>
        {imgLoaded ? (
          <ImageContainer>
            <CustomImg onClick={handleImgClick} src={imgSrc}></CustomImg>
            <ImageInfo>IN[{nodeHovered.backlinkCountContent}] OUT[{nodeHovered.forwardlinkCount}]</ImageInfo>
          </ImageContainer>
        ):(
          <Skeleton 
            variant='rectangular' 
            sx={{
              bgcolor: 'rgba(255,255,255,0.3)',
              borderRadius: '10px'
            }} 
            width={100} 
            height={100*5/4}></Skeleton>
        )}
        <ActionButtons></ActionButtons>
      </RowContainer>
      <Contents>
        {nodeHovered?.additionalInfo?.split('\n')
        .filter((info, index) => index < 10)
        .map((info) => {
          return <Text>{info}</Text>
        })}
      </Contents>
    </CardContainer>
    </>
  )
}

export default React.memo(React.forwardRef(Graph2D));