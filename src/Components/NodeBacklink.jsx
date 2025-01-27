import React from 'react';
import styled from 'styled-components';
import {getBacklinksByContentId} from '../js/serverApi.js';
import {expandNetworkData} from '../js/dataHandlers.js';
import {
  RowContainer,
  Node,
  Title,
  DelButton,
  TimeStamp
} from './NodeStyles.js'


function NodeBacklink(props) {
  const {
    node, 
    removeNode, 
    expandNode,
    setActiveExpandedNodeId
  } = props;
  const [isShowAll, setIsShowAll] = React.useState(false);
  const onClickTitle = React.useCallback(async (event) => {
    expandNode(node)
  }, [node])
  const toggleShowsAllBacklinks = React.useCallback((event) => {
    const {id} = event.target;
    setIsShowAll( async (isShowAll) => {
      const includeOnlyContents = isShowAll;
      const rows = await getBacklinksByContentId(id)
      setLastNetworkData((lastNetworkData) => {
        const newNetworkData = expandNetworkData(rows, id, lastNetworkData, includeOnlyContents);
        return newNetworkData
      })
    })
  }, [])
  return (
    <RowContainer>
      <Node key={node.id}>
        <Title
          id={node.id}
          onClick={onClickTitle}
        >
          {node.text}
        </Title>
        <DelButton id={node.id} onClick={removeNode}>
          [Del]
        </DelButton>
      </Node>
      <TimeStamp
        onClick={toggleShowsAllBacklinks}
        id={node.id}
      >
        backlinks [{node.backlinkCount}]
      </TimeStamp>
    </RowContainer>
  )
}

export default React.memo(NodeBacklink)